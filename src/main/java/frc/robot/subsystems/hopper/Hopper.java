package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public Alert indexerMotorConnectedAlert =
      new Alert("Critical", "Indexer Motor Disconnected", AlertType.kError);

  public HopperIO io;
  public HopperIOInputsAutoLogged inputs;

  public Hopper(HopperIO io) {
    this.io = io;
    this.inputs = new HopperIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    AlertUtils.processCriticalAlert(indexerMotorConnectedAlert, !inputs.indexerTalonConnected);
    // AlertUtils.processCriticalAlert(indexerMotorConnectedAlert,
    // !inputs.topIndexerTalonConnected);
  }

  public Command spinIndexer() {
    return runOnce(
        () -> io.setIndexerSpeed(RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get())));
  }

  public Command stopIndexer() {
    return runOnce(() -> io.setIndexerSpeed(RotationsPerSecond.of(0)));
  }

  public Command spinTopIndexer() {
    return runOnce(
        () ->
            io.setTopIndexerSpeed(RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get())));
  }

  public Command stopTopIndexer() {
    return runOnce(() -> io.setTopIndexerSpeed(RotationsPerSecond.of(0)));
  }

  public Command spinFullIndexer() {
    return runOnce(
        () -> {
          io.setIndexerSpeed(RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get()));
          io.setTopIndexerSpeed(RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get()));
        });
  }

  public Command spinFullIndexer(AngularVelocity indexer, AngularVelocity topIndexer) {
    return runOnce(
        () -> {
          io.setIndexerSpeed(indexer);
          io.setTopIndexerSpeed(topIndexer);
        });
  }

  public Command oscillateTopIndexer() {
    if (!HopperConstants.USE_TOP_INDEXER) {
      return Commands.none();
    }
    return Commands.repeatingSequence(
        runOnce(() -> io.setTopIndexerSpeed(RotationsPerSecond.of(0))),
        Commands.waitSeconds(0.2),
        runOnce(
            () ->
                io.setTopIndexerSpeed(
                    RotationsPerSecond.of(-HopperConstants.TOP_INDEXER_SPEED.get()))),
        Commands.waitSeconds(0.7));
  }

  public Command stopFullIndexer() {
    return runOnce(
        () -> {
          io.setIndexerSpeed(RotationsPerSecond.of(0));
          io.setTopIndexerSpeed(RotationsPerSecond.of(0));
        });
  }

  public void setIndexerSpeed(AngularVelocity speed) {
    io.setIndexerSpeed(speed);
  }

  public void setTopIndexerSpeed(AngularVelocity speed) {
    io.setTopIndexerSpeed(speed);
  }
}
