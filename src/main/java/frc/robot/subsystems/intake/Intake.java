package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public Alert intakeMotorConnectedAlert =
      new Alert("Critical", "Intake Motor Disconnected", AlertType.kError);

  public IntakeIO io;
  public IntakeIOInputsAutoLogged inputs;

  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    AlertUtils.processCriticalAlert(intakeMotorConnectedAlert, !inputs.intakeTalonConnected);
  }

  public Command spinIntake() {
    return runOnce(
        () -> {
          io.setIntakeVoltage(Volts.of(IntakeConstants.INTAKE_VOLTAGE.get()));
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          io.setIntakeVoltage(Volts.of(0));
        });
  }

  public Command reverseIntake() {
    return runOnce(
        () -> {
          io.setIntakeVoltage(Volts.of(-IntakeConstants.INTAKE_VOLTAGE.get()));
        });
  }
}
