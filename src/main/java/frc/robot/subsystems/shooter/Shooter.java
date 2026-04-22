package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.AlertUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public ShooterIO io;
  public ShooterIOInputsAutoLogged inputs;

  public AngularVelocity targetFlywheelVelocity;
  public AngularVelocity targetFeederVelocity;

  /** True when flywheel was at or above the 4 % threshold last loop. */
  private boolean prevAboveThreshold = false;

  /** Latches true for exactly one loop when velocity first dips ≥4 % below target. */
  private boolean ballShotDetected = false;

  /** FPGA timestamp of the last beam break trigger. Negative means never triggered. */
  private double lastBeamBreakTimestampSec = -1.0;

  public Alert shooterMotorConnectedAlert =
      new Alert("Critical", "Shooter Flywheel Motor Disconnected", AlertType.kError);
  public Alert feederMotorConnectedAlert =
      new Alert("Critical", "Shooter Feeder Motor Disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();

    this.targetFlywheelVelocity = RotationsPerSecond.of(0);
    this.targetFeederVelocity = RotationsPerSecond.of(0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Feeder velocity compensation: boost flywheel speed proportional to feeder deficit
    // AngularVelocity compensatedFlywheelVelocity = targetFlywheelVelocity;
    // if (targetFlywheelVelocity.in(RotationsPerSecond) > 0
    //     && targetFeederVelocity.in(RotationsPerSecond) > 0) {
    //   double targetRPS = targetFeederVelocity.in(RotationsPerSecond);
    //   double feederError = targetRPS - inputs.feederVelocity.in(RotationsPerSecond);
    //   double compensation = 0;
    //   if (feederError > 0.15 * targetRPS) {
    //     compensation = feederError * ShooterConstants.FEEDER_COMPENSATION_GAIN.get();
    //   }
    //   compensatedFlywheelVelocity =
    //       RotationsPerSecond.of(targetFlywheelVelocity.in(RotationsPerSecond) + compensation);
    //   io.setFlywheelSpeed(compensatedFlywheelVelocity);
    // }

    // Ball-shot detection: fires for exactly one loop when flywheel velocity first dips ≥4% below
    // target. PID/FF runs continuously — no voltage override.
    double targetRPS = targetFlywheelVelocity.in(RotationsPerSecond);
    if (targetRPS > 0) {
      double currentRPS = inputs.flywheelVelocity.in(RotationsPerSecond);
      boolean aboveThreshold =
          currentRPS >= targetRPS * (1.0 - ShooterConstants.FLYWHEEL_BANG_THROUGH_THRESHOLD);
      ballShotDetected = prevAboveThreshold && !aboveThreshold;
      prevAboveThreshold = aboveThreshold;
      io.setFlywheelSpeed(targetFlywheelVelocity);
    } else {
      prevAboveThreshold = false;
      ballShotDetected = false;
    }

    if (inputs.beamBreakTriggered) {
      lastBeamBreakTimestampSec = Timer.getFPGATimestamp();
    }

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("Shooter/velocityBelowThreshold", !prevAboveThreshold);
      Logger.recordOutput("Shooter/ballExitedFlywheel", ballExitedFlywheel());
      Logger.recordOutput("Shooter/beamBreakTriggered", inputs.beamBreakTriggered);
      Logger.recordOutput("Shooter/isShooting", isShooting());
    }
    Logger.recordOutput("Shooter/targetVelocity", targetFlywheelVelocity.in(RadiansPerSecond));
    Logger.recordOutput("Shooter/targetFeederVelocity", targetFeederVelocity.in(RadiansPerSecond));

    // update alerts based on motor connection status
    AlertUtils.processCriticalAlert(shooterMotorConnectedAlert, !inputs.flywheelTalonConnected);
    AlertUtils.processCriticalAlert(feederMotorConnectedAlert, !inputs.feederTalonConnected);
  }

  /**
   * Returns true for exactly one loop when the flywheel velocity first drops ≥4% below the target,
   * indicating a ball just entered (and is loading) the flywheel.
   */
  public boolean ballExitedFlywheel() {
    return ballShotDetected;
  }

  /** Seconds since the beam break last fired; {@link Double#MAX_VALUE} if never triggered. */
  public double getTimeSinceLastBeamBreakSec() {
    if (lastBeamBreakTimestampSec < 0) return Double.MAX_VALUE;
    return Timer.getFPGATimestamp() - lastBeamBreakTimestampSec;
  }

  /** Returns true if the beam break has been triggered within the last 0.4 s. */
  public boolean isShooting() {
    return lastBeamBreakTimestampSec >= 0
        && (Timer.getFPGATimestamp() - lastBeamBreakTimestampSec)
            < ShooterConstants.SHOOTING_WINDOW_SECONDS;
  }

  public Command setFlywheelRPM(Supplier<AngularVelocity> speed) {
    return runOnce(
        () -> {
          setFlywheelRPM(speed.get());
        });
  }

  public void setFlywheelRPM(AngularVelocity speed) {
    this.targetFlywheelVelocity = speed;
  }

  public Command setFlywheelVoltage(Supplier<Voltage> voltage) {
    return runOnce(
        () -> {
          setFlywheelVoltage(voltage.get());
        });
  }

  public void setFlywheelVoltage(Voltage voltage) {
    io.setFlywheelVoltage(voltage);
    this.targetFlywheelVelocity = RotationsPerSecond.of(0);
    this.targetFeederVelocity = RotationsPerSecond.of(0);
  }

  public Command setFeederVoltage(Supplier<Voltage> voltage) {
    return runOnce(() -> setFeederVoltage(voltage.get()));
  }

  public void setFeederVoltage(Voltage voltage) {
    io.setFeederVoltage(voltage);
    this.targetFeederVelocity = RotationsPerSecond.of(0);
  }

  public Command setFeederSpeed(Supplier<AngularVelocity> speed) {
    return runOnce(() -> setFeederSpeed(speed.get()));
  }

  public void setFeederSpeed(AngularVelocity speed) {
    io.setFeederSpeed(speed);
    this.targetFeederVelocity = speed;
  }

  public boolean flywheelUpToSpeed() {
    return inputs.flywheelVelocity.isNear(
        this.targetFlywheelVelocity, ShooterConstants.FLYWHEEL_TOLERANCE_BEFORE_SHOT);
  }

  public boolean flywheelUpToSpeed(AngularVelocity tolerance) {
    return inputs.flywheelVelocity.isNear(this.targetFlywheelVelocity, tolerance);
  }

  public boolean feederUpToSpeed() {
    return inputs.feederVelocity.isNear(
        this.targetFeederVelocity, ShooterConstants.FEEDER_TOLERANCE_BEFORE_SHOT);
  }

  public AngularVelocity getCurrentFlywheelSpeed() {
    return inputs.flywheelVelocity;
  }

  public void setDefenseMode(boolean active) {
    io.setDefenseMode(active);
  }
}
