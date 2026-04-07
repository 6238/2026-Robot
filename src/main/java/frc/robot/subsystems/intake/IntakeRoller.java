package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import frc.robot.util.BatteryLogger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
  public Alert intakeMotorConnectedAlert =
      new Alert("Critical", "Intake Motor Disconnected", AlertType.kError);

  public IntakeRollerIO io;
  public IntakeRollerIOInputsAutoLogged inputs;

  // Supplier for pivot angle in degrees; roller is suppressed above ROLLER_PIVOT_CUTOFF_DEGREES
  private DoubleSupplier pivotAngleDegSupplier = () -> 0.0;

  public static final double ROLLER_PIVOT_CUTOFF_DEGREES = 80.0;

  // Jam prevention state
  private AngularVelocity desiredIntakeVelocity = RotationsPerSecond.of(0);
  private boolean isJamReversing = false;
  private final Timer jamReverseTimer = new Timer();
  private final Debouncer stallDebouncer =
      new Debouncer(IntakeConstants.STALL_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Runnable onJamDetected;

  private BatteryLogger batteryLogger;

  public void setBatteryLogger(BatteryLogger batteryLogger) {
    this.batteryLogger = batteryLogger;
  }

  public void setPivotAngleSupplier(DoubleSupplier supplier) {
    this.pivotAngleDegSupplier = supplier;
  }

  public IntakeRoller(IntakeRollerIO io) {
    this(io, () -> {});
  }

  public IntakeRoller(IntakeRollerIO io, Runnable onJamDetected) {
    this.io = io;
    this.inputs = new IntakeRollerIOInputsAutoLogged();
    this.onJamDetected = onJamDetected;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);

    Logger.recordOutput(
        "IntakeRoller/currentVelocity", inputs.intakeVelocity.in(RotationsPerSecond));

    if (batteryLogger != null) {
      batteryLogger.reportCurrentUsage("Intake/Roller", inputs.intakeSupplyCurrent.in(Amps));
    }

    AlertUtils.processCriticalAlert(intakeMotorConnectedAlert, !inputs.intakeTalonConnected);

    // Jam prevention
    boolean stallCondition =
        desiredIntakeVelocity.in(RotationsPerSecond) > 0
            && Math.abs(inputs.intakeVelocity.in(RotationsPerSecond))
                < IntakeConstants.STALL_VELOCITY_THRESHOLD_RPS
            && inputs.intakeSupplyCurrent.in(Amps) > IntakeConstants.STALL_CURRENT_THRESHOLD_AMPS;

    boolean stalled = stallDebouncer.calculate(stallCondition);

    if (stalled && !isJamReversing) {
      isJamReversing = true;
      jamReverseTimer.restart();
      io.setIntakeVoltage(Volts.of(-12.0));
      onJamDetected.run();
    }

    if (isJamReversing) {
      boolean velocityRecovered =
          Math.abs(inputs.intakeVelocity.in(RotationsPerSecond))
              > IntakeConstants.STALL_VELOCITY_THRESHOLD_RPS;
      if (velocityRecovered
          && jamReverseTimer.hasElapsed(IntakeConstants.JAM_REVERSE_MIN_DURATION_SECONDS)) {
        isJamReversing = false;
        io.setIntakeVelocity(desiredIntakeVelocity);
      }
    }

    // Suppress roller when pivot is above cutoff angle (don't fight jam reversal)
    if (pivotAngleDegSupplier.getAsDouble() > ROLLER_PIVOT_CUTOFF_DEGREES && !isJamReversing) {
      io.setIntakeVoltage(Volts.of(0));
    }

    Logger.recordOutput("IntakeRoller/isJamReversing", isJamReversing);
  }

  public Command spinIntake() {
    return runOnce(
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(IntakeConstants.INTAKE_SPEED.get());
          if (!isJamReversing) {
            io.setIntakeVelocity(desiredIntakeVelocity);
          }
        });
  }

  /** Runs the intake until cancelled — suitable for toggle bindings. */
  public Command runIntake() {
    return startEnd(
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(IntakeConstants.INTAKE_SPEED.get());
          if (!isJamReversing) io.setIntakeVelocity(desiredIntakeVelocity);
        },
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(0);
          isJamReversing = false;
          io.setIntakeVoltage(Volts.of(0));
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(0);
          isJamReversing = false;
          io.setIntakeVoltage(Volts.of(0));
        });
  }

  public void spin() {
    desiredIntakeVelocity = RotationsPerSecond.of(IntakeConstants.INTAKE_SPEED.get());
    if (!isJamReversing) io.setIntakeVelocity(desiredIntakeVelocity);
  }

  public void stop() {
    desiredIntakeVelocity = RotationsPerSecond.of(0);
    isJamReversing = false;
    io.setIntakeVoltage(Volts.of(0));
  }

  public Command reverseIntake() {
    return runOnce(
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(-IntakeConstants.INTAKE_SPEED.get());
          isJamReversing = false;
          io.setIntakeVelocity(desiredIntakeVelocity);
        });
  }
}
