package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean indexerTalonConnected = false;
    public boolean topIndexerTalonConnected = false;

    public AngularVelocity indexerVelocity = RotationsPerSecond.of(0.0);
    public Current indexerSupplyCurrent = Amps.of(0.0);
    public Voltage indexerAppliedVoltage = Volts.of(0.0);

    public AngularVelocity topIndexerVelocity = RotationsPerSecond.of(0.0);
    public Current topIndexerSupplyCurrent = Amps.of(0.0);
    public Voltage topIndexerAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(HopperIOInputs inputs) {}
  ;

  public default void setIndexerSpeed(AngularVelocity speed) {}
  ;

  public default void setTopIndexerSpeed(AngularVelocity speed) {}
  ;

  public default void setTopIndexerVoltage(Voltage voltage) {}
  ;

  /** Reconfigures indexer current limits. No-op in sim/test. */
  public default void setDefenseMode(boolean active) {}
}
