package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  public Alert indexerMotorConnectedAlert =
      new Alert("Critical", "Indexer Motor Disconnected", AlertType.kError);

  public HopperIO io;
  public HopperIOInputsAutoLogged inputs;

  private AngularVelocity targetTopIndexerVelocity = RotationsPerSecond.of(0);
  private final Debouncer topIndexerJamDebouncer = new Debouncer(0.225, DebounceType.kRising);
  private boolean isUnjamming = false;

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

    boolean topIndexerShouldRun = targetTopIndexerVelocity.in(RotationsPerSecond) > 0;
    boolean topIndexerStalled =
        inputs.topIndexerVelocity.isNear(
            RotationsPerSecond.of(0), HopperConstants.TOP_INDEXER_JAM_THRESHOLD);
    boolean jammed = topIndexerJamDebouncer.calculate(topIndexerShouldRun && topIndexerStalled);

    if (jammed && !isUnjamming) {
      CommandScheduler.getInstance().schedule(unjamTopIndexer());
    }
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
        () -> {
          targetTopIndexerVelocity = RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get());
          io.setTopIndexerSpeed(targetTopIndexerVelocity);
        });
  }

  public Command stopTopIndexer() {
    return runOnce(
        () -> {
          targetTopIndexerVelocity = RotationsPerSecond.of(0);
          io.setTopIndexerSpeed(targetTopIndexerVelocity);
        });
  }

  public Command spinFullIndexer() {
    return runOnce(
        () -> {
          io.setIndexerSpeed(RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get()));
          targetTopIndexerVelocity = RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get());
          io.setTopIndexerSpeed(targetTopIndexerVelocity);
        });
  }

  public Command spinFullIndexer(AngularVelocity indexer, AngularVelocity topIndexer) {
    return runOnce(
        () -> {
          io.setIndexerSpeed(indexer);
          targetTopIndexerVelocity = topIndexer;
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

  public Command unjamTopIndexer() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              isUnjamming = true;
              io.setTopIndexerVoltage(Volts.of(-12));
            }),
        Commands.waitSeconds(0.15),
        Commands.runOnce(
            () -> {
              io.setTopIndexerSpeed(targetTopIndexerVelocity);
              isUnjamming = false;
            }));
  }

  public Command stopFullIndexer() {
    return runOnce(
        () -> {
          io.setIndexerSpeed(RotationsPerSecond.of(0));
          targetTopIndexerVelocity = RotationsPerSecond.of(0);
          io.setTopIndexerSpeed(RotationsPerSecond.of(0));
        });
  }

  public void setIndexerSpeed(AngularVelocity speed) {
    io.setIndexerSpeed(speed);
  }

  public void setTopIndexerSpeed(AngularVelocity speed) {
    targetTopIndexerVelocity = speed;
    io.setTopIndexerSpeed(speed);
  }

  public void setDefenseMode(boolean active) {
    io.setDefenseMode(active);
  }
}
