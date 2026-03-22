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
  public AngularVelocity targetFeederVelocity;

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
    AngularVelocity compensatedFlywheelVelocity = targetFlywheelVelocity;
    if (targetFlywheelVelocity.in(RotationsPerSecond) > 0
        && targetFeederVelocity.in(RotationsPerSecond) > 0) {
      double targetRPS = targetFeederVelocity.in(RotationsPerSecond);
      double feederError = targetRPS - inputs.feederVelocity.in(RotationsPerSecond);
      double compensation = 0;
      if (feederError > 0.15 * targetRPS) {
        compensation = feederError * ShooterConstants.FEEDER_COMPENSATION_GAIN.get();
      }
      compensatedFlywheelVelocity =
          RotationsPerSecond.of(targetFlywheelVelocity.in(RotationsPerSecond) + compensation);
      io.setFlywheelSpeed(compensatedFlywheelVelocity);
    }

    Logger.recordOutput("Shooter/targetVelocity", targetFlywheelVelocity.in(RotationsPerSecond));
    Logger.recordOutput(
        "Shooter/compensatedVelocity", compensatedFlywheelVelocity.in(RotationsPerSecond));
    Logger.recordOutput("Shooter/currentVelocity", inputs.flywheelVelocity.in(RotationsPerSecond));

    // update alerts based on motor connection status
    AlertUtils.processCriticalAlert(shooterMotorConnectedAlert, !inputs.flywheelTalonConnected);
    AlertUtils.processCriticalAlert(feederMotorConnectedAlert, !inputs.feederTalonConnected);
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

  public AngularVelocity getCurrentFlywheelSpeed() {
    return inputs.flywheelVelocity;
  }
}
