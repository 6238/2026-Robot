package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public Alert climberMotorConnectedAlert =
      new Alert("Critical", "Climber Motor Disconnected", AlertType.kError);

  public ClimberIO io;
  public ClimberIOInputsAutoLogged inputs;

  public Climber(ClimberIO io) {
    this.io = io;
    this.inputs = new ClimberIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    AlertUtils.processCriticalAlert(climberMotorConnectedAlert, !inputs.climberTalonConnected);
  }

  public Command setPosition(Distance position) {
    return runOnce(() -> {
        io.setPosition(position);
    });
  }

  public Distance getPosition() {
    return inputs.climberPosition;
  }
}
