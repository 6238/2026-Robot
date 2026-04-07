package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import frc.robot.util.BatteryLogger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  public Alert intakeArmMotorConnectedAlert =
      new Alert("Critical", "Intake Arm Motor Disconnected", AlertType.kError);

  public IntakePivotIO io;
  public IntakePivotIOInputsAutoLogged inputs;

  public Angle targetAngle = Degrees.of(-100);
  private boolean isBrakeMode = true;

  // DRS (Dynamic Recovery System) — intake lower-block detection
  private double drsStuckTimer = 0.0;
  private boolean drsTriggered = false;
  // Ignore DRS for 0.3 s after a new setpoint is commanded (motor inrush looks like a block)
  private double drsIgnoreTimer = 0.0;
  private static final double DRS_IGNORE_SECONDS = 0.25;

  private BatteryLogger batteryLogger;

  public void setBatteryLogger(BatteryLogger batteryLogger) {
    this.batteryLogger = batteryLogger;
  }

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.inputs = new IntakePivotIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    if (batteryLogger != null) {
      batteryLogger.reportCurrentUsage("Intake/Pivot", inputs.intakeArmSupplyCurrent.in(Amps));
    }

    AlertUtils.processCriticalAlert(intakeArmMotorConnectedAlert, !inputs.intakeArmTalonConnected);

    // DRS detection: pivot is targeting the ground setpoint, current is high, and it's stuck
    boolean targetingDown =
        Math.abs(targetAngle.in(Degrees) - IntakeConstants.INTAKE_DOWN_VALUE.get())
            < IntakeConstants.DRS_TARGET_TOLERANCE_DEGREES;
    boolean highCurrent =
        inputs.intakeArmSupplyCurrent.in(Amps) > IntakeConstants.DRS_CURRENT_THRESHOLD_AMPS.get();
    boolean stuckAboveTarget =
        inputs.intakeArmPosition.in(Degrees) - IntakeConstants.INTAKE_DOWN_VALUE.get()
            > IntakeConstants.DRS_ANGLE_ERROR_THRESHOLD_DEGREES.get();
    if (drsIgnoreTimer > 0.0) {
      drsIgnoreTimer -= 0.02;
    }
    boolean drsCondition = targetingDown && stuckAboveTarget && drsIgnoreTimer <= 0.0;
    if (drsCondition) {
      drsStuckTimer += 0.02;
      if (drsStuckTimer >= IntakeConstants.DRS_STUCK_TIMEOUT_SECONDS) {
        drsTriggered = true;
      }
    } else {
      drsStuckTimer = 0.0;
      drsTriggered = false;
    }

    Logger.recordOutput("Intake/currentAngle", inputs.intakeArmPosition.in(Degrees));
    Logger.recordOutput("Intake/targetAngle", targetAngle.in(Degrees));
    Logger.recordOutput("IntakePivot/DRS/Triggered", drsTriggered);
    Logger.recordOutput("IntakePivot/DRS/Condition", drsCondition);
    Logger.recordOutput("IntakePivot/DRS/StuckTimerSecs", drsStuckTimer);
    Logger.recordOutput("IntakePivot/DRS/IgnoreTimerSecs", drsIgnoreTimer);
  }

  /** Returns true when a lower-block has been detected and DRS should fire. */
  public boolean isDRSTriggered() {
    return drsTriggered;
  }

  /** Clears the DRS latch and resets the stuck timer. Call after recovery is complete. */
  public void resetDRS() {
    drsTriggered = false;
    drsStuckTimer = 0.0;
    drsIgnoreTimer = DRS_IGNORE_SECONDS;
  }

  /** Force-triggers DRS immediately. For testing only. */
  public void forceTriggerDRS() {
    drsTriggered = true;
  }

  public Command setIntakeAngle(Supplier<Angle> angleSupplier) {
    return runOnce(
        () -> {
          Angle newAngle = angleSupplier.get();
          boolean changed = Math.abs(newAngle.in(Degrees) - targetAngle.in(Degrees)) > 2.0;
          targetAngle = newAngle;
          Logger.recordOutput("Intake/targetAngle", targetAngle);
          io.setIntakePosition(newAngle);
          if (changed) {
            drsIgnoreTimer = DRS_IGNORE_SECONDS;
            drsStuckTimer = 0.0;
          }
        });
  }

  public void setAngle(Angle angle) {
    boolean changed = Math.abs(angle.in(Degrees) - targetAngle.in(Degrees)) > 2.0;
    targetAngle = angle;
    io.setIntakePosition(angle);
    if (changed) {
      drsIgnoreTimer = DRS_IGNORE_SECONDS;
      drsStuckTimer = 0.0;
    }
  }

  public Command setIntakeArmVoltage(Supplier<Voltage> voltageSupplier) {
    return runOnce(() -> io.setIntakeArmVoltage(voltageSupplier.get()));
  }

  public Command reset() {
    return runOnce(() -> io.resetArmAngle());
  }

  /**
   * Runs the pivot at a small constant voltage until the current spikes (hard-stop contact), taking
   * up backlash before autonomous. Times out after PIVOT_PRELOAD_TIMEOUT_SECONDS.
   */
  public Command preloadPivot() {
    return Commands.run(
            () -> io.setIntakeArmVoltage(Volts.of(IntakeConstants.PIVOT_PRELOAD_VOLTAGE_VOLTS)),
            this)
        .until(
            () ->
                inputs.intakeArmSupplyCurrent.in(Amps)
                    >= IntakeConstants.PIVOT_PRELOAD_CURRENT_THRESHOLD_AMPS)
        .withTimeout(IntakeConstants.PIVOT_PRELOAD_TIMEOUT_SECONDS)
        .finallyDo(() -> io.setIntakeArmVoltage(Volts.of(0)));
  }

  /**
   * Slowly drives the pivot upward. If the current spikes above the threshold the pivot backs off
   * briefly before resuming. Will not exceed INTAKE_UP_VALUE + CRAWL_MAX_OFFSET_DEGREES.
   */
  public Command crawlUp() {
    double[] backoffTimer = {0.0};
    boolean[] isBackingOff = {false};
    boolean[] backoffTriggered = {false};
    return Commands.run(
            () -> {
              double positionDeg = inputs.intakeArmPosition.in(Degrees);
              double maxDeg =
                  IntakeConstants.INTAKE_UP_VALUE.get() + IntakeConstants.CRAWL_MAX_OFFSET_DEGREES;

              if (positionDeg >= maxDeg) {
                io.setIntakeArmVoltage(Volts.of(0));
                return;
              }

              // cos(angle) is 1 at 0° (horizontal, full gravity load) and 0 at 90° (vertical)
              double cosScale = Math.cos(Math.toRadians(positionDeg));
              double scaledVoltage =
                  Math.max(
                      IntakeConstants.CRAWL_UP_VOLTAGE_VOLTS.get() * cosScale,
                      IntakeConstants.CRAWL_UP_VOLTAGE_MIN_VOLTS.get());
              double scaledCurrentThreshold =
                  Math.max(
                      IntakeConstants.CRAWL_CURRENT_THRESHOLD_AMPS.get() * cosScale,
                      IntakeConstants.CRAWL_CURRENT_THRESHOLD_MIN_AMPS.get());

              if (isBackingOff[0]) {
                backoffTimer[0] += 0.02;
                io.setIntakeArmVoltage(
                    Volts.of(-IntakeConstants.CRAWL_BACKOFF_VOLTAGE_VOLTS.get()));
                if (backoffTimer[0] >= IntakeConstants.CRAWL_BACKOFF_DURATION_SECONDS.get()
                    || positionDeg <= IntakeConstants.INTAKE_DOWN_VALUE.get()) {
                  isBackingOff[0] = false;
                  backoffTimer[0] = 0.0;
                }
              } else {
                boolean aboveDeadzone =
                    positionDeg > IntakeConstants.INTAKE_DOWN_VALUE.get() + 10.0;
                if (aboveDeadzone
                    && inputs.intakeArmSupplyCurrent.in(Amps) >= scaledCurrentThreshold) {
                  isBackingOff[0] = true;
                  backoffTriggered[0] = true;
                  backoffTimer[0] = 0.0;
                } else {
                  backoffTriggered[0] = false;
                  io.setIntakeArmVoltage(Volts.of(scaledVoltage));
                }
              }

              Logger.recordOutput("IntakePivot/CrawlIsBackingOff", isBackingOff[0]);
              Logger.recordOutput("IntakePivot/CrawlBackoffTriggered", backoffTriggered[0]);
              Logger.recordOutput("IntakePivot/CrawlScaledVoltage", scaledVoltage);
              Logger.recordOutput(
                  "IntakePivot/CrawlScaledCurrentThreshold", scaledCurrentThreshold);
            },
            this)
        .finallyDo(
            () -> {
              io.setIntakeArmVoltage(Volts.of(0));
              Logger.recordOutput("IntakePivot/CrawlIsBackingOff", false);
              Logger.recordOutput("IntakePivot/CrawlBackoffTriggered", false);
            });
  }

  public Command toggleBrakeMode() {
    return runOnce(
            () -> {
              isBrakeMode = !isBrakeMode;
              io.setBrakeMode(isBrakeMode);
              Logger.recordOutput("IntakePivot/IsBrakeMode", isBrakeMode);
            })
        .ignoringDisable(true);
  }
}
