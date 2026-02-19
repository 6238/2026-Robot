package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  // Physics constants for intake roller (flywheel)
  private static final double ROLLER_MASS_KG = 0.5; // Estimated mass of roller
  private static final double ROLLER_RADIUS_M = Units.inchesToMeters(2.0); // 4" diameter roller

  // Physics constants for intake arm
  private static final double ARM_LENGTH_M = Units.inchesToMeters(18.0); // 18" arm length
  private static final double ARM_MASS_KG = 4.0; // Estimated mass of arm
  private static final double ARM_MOI = SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG);

  // Moment of inertia for roller: I = 0.5 * m * r^2
  private static final double ROLLER_MOI = 0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M;

  // Create TalonFX objects for simulation
  private final TalonFX intakeTalon;
  private final TalonFX intakeArmTalon;

  // Simulated motor controllers
  private final FlywheelSim intakeRollerSim;
  private final SingleJointedArmSim intakeArmSim;

  // Phoenix 6 simulation states
  private final TalonFXSimState intakeSimState;
  private final TalonFXSimState intakeArmSimState;

  // Control requests
  private final MotionMagicVoltage motionMagicVoltageArm = new MotionMagicVoltage(0).withSlot(0);
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public IntakeIOSim() {
    // Create TalonFX objects (these exist only in simulation)
    intakeTalon = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS);
    intakeArmTalon = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_ID, IntakeConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;
    intakeConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_DIRECTION;
    intakeConfig.Slot0 = IntakeConstants.INTAKE_GAINS.toSlot0Configs();

    TalonFXConfiguration intakeArmConfig = new TalonFXConfiguration();
    intakeArmConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_ARM_GEARING;
    intakeArmConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_ARM_MOTOR_DIRECTION;
    intakeArmConfig.Slot0 = IntakeConstants.INTAKE_ARM_GAINS.toSlot0Configs();
    intakeArmConfig.MotionMagic = IntakeConstants.INTAKE_ARM_MOTION_MAGIC_CONFIGS;

    tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES, () -> intakeTalon.getConfigurator().apply(intakeConfig));
    tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES,
        () -> intakeArmTalon.getConfigurator().apply(intakeArmConfig));

    // Get Phoenix 6 simulation states
    intakeSimState = intakeTalon.getSimState();
    intakeArmSimState = intakeArmTalon.getSimState();

    // Set supply voltage for sim
    intakeSimState.setSupplyVoltage(12.0);
    intakeArmSimState.setSupplyVoltage(12.0);

    // Create intake roller flywheel simulation
    intakeRollerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), ROLLER_MOI, IntakeConstants.INTAKE_GEARING),
            DCMotor.getKrakenX60(1));

    // Create intake arm simulation
    intakeArmSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            IntakeConstants.INTAKE_ARM_GEARING,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG),
            ARM_LENGTH_M,
            -Math.PI, // Minimum angle
            Math.PI, // Maximum angle
            true, // Simulate gravity
            IntakeConstants.INTAKE_START_VALUE.in(Radians));

    // Set initial arm position
    intakeArmTalon.setPosition(IntakeConstants.INTAKE_START_VALUE);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Get motor voltages from Phoenix 6 sim states
    double intakeVoltage = intakeSimState.getMotorVoltage();
    double intakeArmVoltage = intakeArmSimState.getMotorVoltage();

    // Update physics simulations
    intakeRollerSim.setInputVoltage(intakeVoltage);
    intakeArmSim.setInputVoltage(intakeArmVoltage);

    intakeRollerSim.update(0.02); // 20ms period
    intakeArmSim.update(0.02);

    // FlywheelSim gives us MECHANISM velocity (after gearing)
    // Convert to ROTOR velocity (before gearing) by multiplying by gear ratio
    // Convert rad/s to rotations/s
    double intakeMechanismRPS = intakeRollerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double intakeRotorRPS = intakeMechanismRPS * IntakeConstants.INTAKE_GEARING;

    intakeSimState.setRotorVelocity(intakeRotorRPS);
    intakeSimState.addRotorPosition(intakeRotorRPS * 0.02);

    // Arm simulation
    double armAngleRad = intakeArmSim.getAngleRads();
    double armVelocityRadPerSec = intakeArmSim.getVelocityRadPerSec();

    // Convert arm velocity to rotor velocity
    double armRotorRPS =
        (armVelocityRadPerSec / (2.0 * Math.PI)) * IntakeConstants.INTAKE_ARM_GEARING;

    intakeArmSimState.setRotorVelocity(armRotorRPS);
    intakeArmSimState.addRotorPosition(armRotorRPS * 0.02);

    // Update inputs
    inputs.intakeTalonConnected = true;
    inputs.intakeArmTalonConnected = true;

    inputs.intakeVelocity = intakeTalon.getVelocity().getValue();
    inputs.intakeAcceleration = intakeTalon.getAcceleration().getValue();
    inputs.intakeAppliedCurrent = intakeTalon.getStatorCurrent().getValue();
    inputs.intakeAppliedVoltage = intakeTalon.getMotorVoltage().getValue();

    inputs.intakeArmPosition = intakeArmTalon.getPosition().getValue();
    inputs.intakeArmVelocity = intakeArmTalon.getVelocity().getValue();
    inputs.intakeArmAcceleration = intakeArmTalon.getAcceleration().getValue();
    inputs.intakeArmAppliedCurrent = intakeArmTalon.getStatorCurrent().getValue();
    inputs.intakeArmAppliedVoltage = intakeArmTalon.getMotorVoltage().getValue();
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    intakeTalon.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setIntakeVelocity(AngularVelocity speed) {
    intakeTalon.setControl(velocityVoltage.withVelocity(speed).withSlot(0));
  }

  @Override
  public void setIntakePosition(Angle targetAngle) {
    intakeArmTalon.setControl(motionMagicVoltageArm.withPosition(targetAngle).withSlot(0));
  }
}
