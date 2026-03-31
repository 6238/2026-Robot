package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakePivotIOSim implements IntakePivotIO {
  // Arm physical constants (estimates for the intake pivot mechanism)
  private static final double ARM_MOI_KG_M2 = 0.04; // 0.5 * 0.5kg * 0.4m^2
  private static final double ARM_LENGTH_M = 0.4;
  private static final double MIN_ANGLE_DEG = -40.0;
  private static final double MAX_ANGLE_DEG = 125.0;
  private static final double START_ANGLE_DEG = 70.0;

  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          IntakeConstants.INTAKE_ARM_GEARING,
          ARM_MOI_KG_M2,
          ARM_LENGTH_M,
          Math.toRadians(MIN_ANGLE_DEG),
          Math.toRadians(MAX_ANGLE_DEG),
          true,
          Math.toRadians(START_ANGLE_DEG));

  private double targetPositionRad = Math.toRadians(START_ANGLE_DEG);
  private double appliedVolts = 0.0;
  private boolean positionControl = false;

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    if (positionControl) {
      double positionErrorRad = targetPositionRad - armSim.getAngleRads();
      double velocityRadPerSec = armSim.getVelocityRadPerSec();
      // Simple PD controller; gains tuned for reasonable sim response
      appliedVolts = 8.0 * positionErrorRad - 0.3 * velocityRadPerSec;
      appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    }

    armSim.setInputVoltage(appliedVolts);
    armSim.update(0.02);

    inputs.intakeArmTalonConnected = true;
    inputs.intakeArmPosition = Degrees.of(Math.toDegrees(armSim.getAngleRads()));
    inputs.intakeArmVelocity =
        RotationsPerSecond.of(armSim.getVelocityRadPerSec() / (2.0 * Math.PI));
    inputs.intakeArmAcceleration = RotationsPerSecondPerSecond.of(0.0);
    inputs.intakeArmSupplyCurrent = Amps.of(Math.abs(armSim.getCurrentDrawAmps()));
    inputs.intakeArmAppliedVoltage = Volts.of(appliedVolts);
  }

  @Override
  public void setIntakePosition(Angle targetAngle) {
    targetPositionRad = Math.toRadians(targetAngle.in(Degrees));
    positionControl = true;
  }

  @Override
  public void setIntakeArmVoltage(Voltage voltage) {
    appliedVolts = voltage.in(Volts);
    positionControl = false;
  }

  @Override
  public void resetArmAngle() {
    targetPositionRad = Math.toRadians(IntakeConstants.INTAKE_DOWN_VALUE.get());
    positionControl = true;
  }
}
