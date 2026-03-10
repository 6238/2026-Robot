package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public Alert intakeMotorConnectedAlert =
      new Alert("Critical", "Intake Motor Disconnected", AlertType.kError);

  public IntakeIO io;
  public IntakeIOInputsAutoLogged inputs;

  public Angle targetAngle = Degrees.of(-100);

  // Jam prevention state
  private AngularVelocity desiredIntakeVelocity = RotationsPerSecond.of(0);
  private boolean isJamReversing = false;
  private final Timer jamReverseTimer = new Timer();
  private final Debouncer stallDebouncer =
      new Debouncer(IntakeConstants.STALL_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final Runnable onJamDetected;

  public Intake(IntakeIO io) {
    this(io, () -> {});
  }

  public Intake(IntakeIO io, Runnable onJamDetected) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
    this.onJamDetected = onJamDetected;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("currentVelocity", inputs.intakeVelocity.in(RotationsPerSecond));

    AlertUtils.processCriticalAlert(intakeMotorConnectedAlert, !inputs.intakeTalonConnected);

    // Jam prevention
    boolean stallCondition =
        desiredIntakeVelocity.in(RotationsPerSecond) > 0
            && Math.abs(inputs.intakeVelocity.in(RotationsPerSecond))
                < IntakeConstants.STALL_VELOCITY_THRESHOLD_RPS
            && inputs.intakeAppliedCurrent.in(Amps) > IntakeConstants.STALL_CURRENT_THRESHOLD_AMPS;

    boolean stalled = stallDebouncer.calculate(stallCondition);

    if (stalled && !isJamReversing) {
      isJamReversing = true;
      jamReverseTimer.restart();
      io.setIntakeVoltage(Volts.of(12.0));
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

    Logger.recordOutput("Intake/isJamReversing", isJamReversing);
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

  public Command stopIntake() {
    return runOnce(
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(0);
          isJamReversing = false;
          io.setIntakeVoltage(Volts.of(0));
        });
  }

  public Command reverseIntake() {
    return runOnce(
        () -> {
          desiredIntakeVelocity = RotationsPerSecond.of(-IntakeConstants.INTAKE_SPEED.get());
          isJamReversing = false;
          io.setIntakeVelocity(desiredIntakeVelocity);
        });
  }

  public Command setIntakeAngle(Supplier<Angle> angleSupplier) {
    return runOnce(
        () -> {
          targetAngle = angleSupplier.get();
          io.setIntakePosition(angleSupplier.get());
        });
  }

  public Command setIntakeArmVoltage(Supplier<Voltage> voltageSupplier) {
    return runOnce(
        () -> {
          io.setIntakeArmVoltage(voltageSupplier.get());
        });
  }

  public Command reset() {
    return runOnce(() -> io.resetArmAngle());
  }
}
