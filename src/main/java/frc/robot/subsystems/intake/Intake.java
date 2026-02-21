package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

  private Angle targetAngle = Degrees.of(-100);

  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("currentVelocity", inputs.intakeVelocity.in(RotationsPerSecond));

    AlertUtils.processCriticalAlert(intakeMotorConnectedAlert, !inputs.intakeTalonConnected);
  }

  public Command spinIntake() {
    return runOnce(
        () -> {
          io.setIntakeVelocity(RotationsPerSecond.of(IntakeConstants.INTAKE_SPEED.get()));
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
          io.setIntakeVelocity(RotationsPerSecond.of(-IntakeConstants.INTAKE_SPEED.get()));
        });
  }

  public Command setIntakeAngle(Supplier<Angle> angleSupplier) {
    return runOnce(
        () -> {
          targetAngle = angleSupplier.get();
          io.setIntakePosition(angleSupplier.get());
        });
  }

  public Command reset() {
    return runOnce(() -> io.resetArmAngle());
  }
}
