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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  public Alert intakeArmMotorConnectedAlert =
      new Alert("Critical", "Intake Arm Motor Disconnected", AlertType.kError);

  public IntakePivotIO io;
  public IntakePivotIOInputsAutoLogged inputs;

  public Angle targetAngle = Degrees.of(-100);
  private boolean isBrakeMode = true;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    this.inputs = new IntakePivotIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

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
                inputs.intakeArmAppliedCurrent.in(Amps)
                    >= IntakeConstants.PIVOT_PRELOAD_CURRENT_THRESHOLD_AMPS)
        .withTimeout(IntakeConstants.PIVOT_PRELOAD_TIMEOUT_SECONDS)
        .finallyDo(() -> io.setIntakeArmVoltage(Volts.of(0)));
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
