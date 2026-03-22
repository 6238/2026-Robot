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
  }

  public Command setIntakeAngle(Supplier<Angle> angleSupplier) {
    return runOnce(
        () -> {
          targetAngle = angleSupplier.get();
          io.setIntakePosition(angleSupplier.get());
        });
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
