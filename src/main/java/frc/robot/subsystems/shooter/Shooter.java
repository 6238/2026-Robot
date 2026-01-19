package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
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

  public Alert shooterMotorConnectedAlert =
      new Alert("Critical", "Shooter Flywheel Motor Disconnected", AlertType.kError);
  public Alert feederMotorConnectedAlert =
      new Alert("Critical", "Shooter Feeder Motor Disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("shooterspeed", inputs.flywheelVelocity.in(RPM));

    // update alerts based on motor connection status
    AlertUtils.updateAlert(shooterMotorConnectedAlert, !inputs.flywheelTalonConnected);
    AlertUtils.updateAlert(feederMotorConnectedAlert, !inputs.feederTalonConnected);
  }

  public Command setFlywheelRPM(Supplier<AngularVelocity> speed) {
    return runOnce(
        () -> {
          Logger.recordOutput("shooter_rpm", speed.get().in(RotationsPerSecond));
          io.setFlywheelSpeed(speed.get());
        });
  }

  public Command setFeederVoltage(Supplier<Voltage> voltage) {
    return runOnce(() -> io.setFeederVoltage(voltage.get()));
  }

  public AngularVelocity getCurrentFlywheelSpeed() {
    return inputs.flywheelVelocity;
  }
}
