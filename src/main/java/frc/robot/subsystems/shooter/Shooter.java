package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public ShooterIO io;
  public ShooterIOInputsAutoLogged inputs;

  public AngularVelocity targetFlywheelVelocity;

  public Alert shooterMotorConnectedAlert =
      new Alert("Critical", "Shooter Flywheel Motor Disconnected", AlertType.kError);
  public Alert feederMotorConnectedAlert =
      new Alert("Critical", "Shooter Feeder Motor Disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();

    this.targetFlywheelVelocity = RotationsPerSecond.of(0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/targetVelocity", targetFlywheelVelocity);

    // update alerts based on motor connection status
    AlertUtils.updateAlert(shooterMotorConnectedAlert, !inputs.flywheelTalonConnected);
    AlertUtils.updateAlert(feederMotorConnectedAlert, !inputs.feederTalonConnected);
  }

  public Command setFlywheelRPM(Supplier<AngularVelocity> speed) {
    return runOnce(
        () -> {
          setFlywheelRPM(speed.get());
        });
  }

  public void setFlywheelRPM(AngularVelocity speed) {
    io.setFlywheelSpeed(speed);
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
  }

  public Command setFeederVoltage(Supplier<Voltage> voltage) {
    return runOnce(() -> setFeederVoltage(voltage.get()));
  }

  public void setFeederVoltage(Voltage voltage) {
    io.setFeederVoltage(voltage);
  }

  public boolean flywheelUpToSpeed() {
    return inputs.flywheelVelocity.isNear(
        this.targetFlywheelVelocity, ShooterConstants.FLYWHEEL_TOLERANCE_BEFORE_SHOT);
  }

  public boolean flywheelUpToSpeed(AngularVelocity tolerance) {
    return inputs.flywheelVelocity.isNear(this.targetFlywheelVelocity, tolerance);
  }

  public AngularVelocity getCurrentFlywheelSpeed() {
    return inputs.flywheelVelocity;
  }
}
