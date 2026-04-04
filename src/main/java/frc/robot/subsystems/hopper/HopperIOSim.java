package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HopperIOSim implements HopperIO {
  private static final double INDEXER_MASS_KG = 0.2;
  private static final double INDEXER_RADIUS_M = 0.025;
  private static final double INDEXER_MOI =
      0.5 * INDEXER_MASS_KG * INDEXER_RADIUS_M * INDEXER_RADIUS_M;

  private final FlywheelSim indexerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), INDEXER_MOI, HopperConstants.INDEXER_GEARING),
          DCMotor.getKrakenX60(1));

  private final FlywheelSim topIndexerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), INDEXER_MOI, HopperConstants.TOP_INDEXER_GEARING),
          DCMotor.getKrakenX60(1));

  private double indexerVolts = 0.0;
  private double topIndexerVolts = 0.0;

  private static double speedToVolts(double rps) {
    if (rps == 0.0) return 0.0;
    return rps * 0.12 + Math.signum(rps) * 0.1;
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    indexerSim.setInputVoltage(indexerVolts);
    topIndexerSim.setInputVoltage(topIndexerVolts);
    indexerSim.update(0.02);
    topIndexerSim.update(0.02);

    inputs.indexerTalonConnected = true;
    inputs.topIndexerTalonConnected = true;

    inputs.indexerVelocity =
        RotationsPerSecond.of(indexerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
    inputs.indexerAcceleration = RotationsPerSecondPerSecond.of(0.0);
    inputs.indexerSupplyCurrent = Amps.of(Math.abs(indexerSim.getCurrentDrawAmps()));
    inputs.indexerAppliedVoltage = Volts.of(indexerVolts);

    inputs.topIndexerVelocity =
        RotationsPerSecond.of(topIndexerSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
    inputs.topIndexerAcceleration = RotationsPerSecondPerSecond.of(0.0);
    inputs.topIndexerSupplyCurrent = Amps.of(Math.abs(topIndexerSim.getCurrentDrawAmps()));
    inputs.topIndexerAppliedVoltage = Volts.of(topIndexerVolts);
  }

  @Override
  public void setIndexerSpeed(AngularVelocity speed) {
    double volts = speedToVolts(speed.in(RotationsPerSecond));
    indexerVolts = volts;
    topIndexerVolts = volts;
  }

  @Override
  public void setTopIndexerSpeed(AngularVelocity speed) {
    topIndexerVolts = speedToVolts(speed.in(RotationsPerSecond));
  }
}
