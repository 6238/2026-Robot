package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

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
        () -> {
          io.setIndexerVoltage(Volts.of(HopperConstants.INDEXER_VOLTAGE.get()));
        });
  }

  public Command stopIndexer() {
    return runOnce(
        () -> {
          io.setIndexerVoltage(Volts.of(0));
        });
  }

  public Command spinTopIndexer() {
    return runOnce(
        () -> {
          io.setTopIndexerVoltage(Volts.of(HopperConstants.TOP_INDEXER_VOLTAGE.get()));
        });
  }

  public Command stopTopIndexer() {
    return runOnce(
        () -> {
          io.setTopIndexerVoltage(Volts.of(1));
        });
  }

  public Command spinFullIndexer() {
    return runOnce(
        () -> {
          io.setIndexerVoltage(Volts.of(HopperConstants.INDEXER_VOLTAGE.get()));
          io.setTopIndexerVoltage(Volts.of(HopperConstants.TOP_INDEXER_VOLTAGE.get()));
        });
  }

  public Command spinFullIndexer(double a, double b) {
    return runOnce(
        () -> {
          io.setIndexerVoltage(Volts.of(a));
          io.setTopIndexerVoltage(Volts.of(b));
        });
  }

  public Command oscillateTopIndexer() {
    return Commands.repeatingSequence(
        runOnce(() -> io.setTopIndexerVoltage(Volts.of(0))),
        Commands.waitSeconds(0.2),
        runOnce(
            () -> io.setTopIndexerVoltage(Volts.of(-HopperConstants.TOP_INDEXER_VOLTAGE.get()))),
        Commands.waitSeconds(0.7));
  }

  public Command stopFullIndexer() {
    return runOnce(
        () -> {
          io.setIndexerVoltage(Volts.of(0));
          io.setTopIndexerVoltage(Volts.of(0));
        });
  }
}
