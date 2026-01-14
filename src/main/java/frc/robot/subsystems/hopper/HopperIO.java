package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean indexerTalonConnected = false;

    public AngularVelocity hopperVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration hopperAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public Current hopperAppliedCurrent = Amps.of(0.0);
    public Voltage hopperAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(HopperIOInputs inputs) {};

  public default void setIndexerVoltage(Voltage voltage) {};
}
