package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    public HopperIO io;
    public HopperIOInputsAutoLogged inputs;

    public Hopper(HopperIO io) {
        this.io = io;
        this.inputs = new HopperIOInputsAutoLogged();
    }

    public Command spinIndexer() {
        return runOnce(() -> {
            io.setIndexerVoltage(Volts.of(HopperConstants.INDEXER_VOLTAGE.get()));
        });
    }

    public Command reverseIndexer() {
        return runOnce(() -> {
            io.setIndexerVoltage(Volts.of(HopperConstants.REVERSE_INDEXER_VOLTAGE.get()));
        });
    }

    public Command stopIndexer() {
        return runOnce(() -> {
            io.setIndexerVoltage(Volts.of(0));
        });
    }
}
