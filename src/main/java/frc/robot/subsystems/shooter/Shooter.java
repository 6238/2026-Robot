package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  public Alert shooterMotorConnectedAlert =
      new Alert("Critical", "Shooter Flywheel Motor Disconnected", AlertType.kError);
  public Alert feederMotorConnectedAlert =
      new Alert("Critical", "Shooter Feeder Motor Disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // update alerts based on motor connection status
    AlertUtils.updateAlert(shooterMotorConnectedAlert, !inputs.flywheelTalonConnected);
    AlertUtils.updateAlert(feederMotorConnectedAlert, !inputs.feederTalonConnected);
  }

  public Command setFlywheelRPM(Supplier<AngularVelocity> rpm) {
    return runOnce(() -> io.setFlywheelRPM(rpm.get()));
  }

  public Command setFeederVoltage(Supplier<Voltage> voltage) {
    return runOnce(() -> io.setFeederVoltage(voltage.get()));
  }
}
