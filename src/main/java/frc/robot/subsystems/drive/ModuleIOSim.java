package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/**
 * MapleSim-backed physics sim implementation of module IO.
 *
 * <p>Matches the existing 6328-style ModuleIO API: - open-loop setters take volts - closed-loop
 * uses local PID (+ drive feedforward) - updateInputs populates ModuleIOInputs and odometry arrays
 */
public class ModuleIOSim implements ModuleIO {
  // Keep the same gains/structure as the original sim
  private static final double DRIVE_KP = 0.2;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;

  // Same intent as your original: (volt * secs) / rotation, converted to V/(rad/s)
  private static final double DRIVE_KV_ROT = 0.91035;
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  // MapleSim simulation reference
  private final SwerveModuleSimulation moduleSimulation;

  // MapleSim generic simulated motor controllers
  private final GenericMotorController driveMotor;
  private final GenericMotorController turnMotor;

  // Control state
  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;

  private final PIDController driveController = new PIDController(DRIVE_KP, 0.0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0.0, TURN_KD);

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private double lastTimestamp = Timer.getTimestamp();
  private double lastDrivePosRad = 0.0;
  private double lastTurnPosRad = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    // Configure generic motor controllers (current limits optional)
    this.driveMotor =
        moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    final Rotation2d steerAbs = moduleSimulation.getSteerAbsoluteFacing();

    final Angle driveWheelPos = moduleSimulation.getDriveWheelFinalPosition();
    final double drivePosRad = driveWheelPos.in(Radians);

    final double turnPosRad = steerAbs.getRadians();

    final double now = Timer.getTimestamp();
    final double dt = Math.max(1e-6, now - lastTimestamp);

    final double driveVelRadPerSec = (drivePosRad - lastDrivePosRad) / dt;
    final double turnVelRadPerSec = (turnPosRad - lastTurnPosRad) / dt;

    lastTimestamp = now;
    lastDrivePosRad = drivePosRad;
    lastTurnPosRad = turnPosRad;

    // run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveVelRadPerSec);
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnPosRad);
    } else {
      turnController.reset();
    }

    final double driveVoltsClamped = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
    final double turnVoltsClamped = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);

    driveMotor.requestVoltage(Volts.of(driveVoltsClamped));
    turnMotor.requestVoltage(Volts.of(turnVoltsClamped));

    inputs.driveConnected = true;
    inputs.drivePositionRad = drivePosRad;
    inputs.driveVelocityRadPerSec = driveVelRadPerSec;
    inputs.driveAppliedVolts = driveVoltsClamped;
    inputs.driveSupplyCurrentAmps = 0.0;

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = steerAbs;
    inputs.turnPosition = steerAbs;

    inputs.turnVelocityRadPerSec = turnVelRadPerSec;
    inputs.turnAppliedVolts = turnVoltsClamped;
    inputs.turnSupplyCurrentAmps = 0.0;

    inputs.odometryTimestamps = new double[] {now};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
