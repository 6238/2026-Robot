package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  // Roller physical constants (dual-motor, small roller)
  private static final double ROLLER_MASS_KG = 0.3;
  private static final double ROLLER_RADIUS_M = 0.025; // ~1" radius
  private static final double ROLLER_MOI = 0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M;

  private final FlywheelSim rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(2), ROLLER_MOI, IntakeConstants.INTAKE_GEARING),
          DCMotor.getKrakenX60(2));

  private double appliedVolts = 0.0;
  private double positionRot = 0.0;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    rollerSim.setInputVoltage(appliedVolts);
    rollerSim.update(0.02);

    double mechanismRPS = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    positionRot += mechanismRPS * 0.02;

    inputs.intakeTalonConnected = true;
    inputs.intakePosition = Rotations.of(positionRot);
    inputs.intakeVelocity = RotationsPerSecond.of(mechanismRPS);
    inputs.intakeAcceleration = RotationsPerSecondPerSecond.of(0.0);
    inputs.intakeSupplyCurrent = Amps.of(Math.abs(rollerSim.getCurrentDrawAmps()));
    inputs.intakeAppliedVoltage = Volts.of(appliedVolts);
    inputs.intakeTemperature = Celsius.of(25.0);
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    appliedVolts = voltage.in(Volts);
  }

  @Override
  public void setIntakeVelocity(AngularVelocity speed) {
    double targetRPS = speed.in(RotationsPerSecond);
    double currentRPS = rollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    appliedVolts = Math.max(-12.0, Math.min(12.0, (targetRPS - currentRPS) * 0.2));
  }
}
