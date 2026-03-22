package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean flywheelTalonConnected = false;
    public boolean feederTalonConnected = false;

    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0);
    public AngularAcceleration flywheelAcceleration = RotationsPerSecondPerSecond.of(0);
    public Current flywheelAppliedCurrent = Amps.of(0);
    public Voltage flywheelAppliedVoltage = Volts.of(0);

    public AngularVelocity feederVelocity = RotationsPerSecond.of(0);
    public AngularAcceleration feederAcceleration = RotationsPerSecondPerSecond.of(0);
    public Current feederAppliedCurrent = Amps.of(0);
    public Voltage feederAppliedVoltage = Volts.of(0);
  }

  public default void setFlywheelSpeed(AngularVelocity rpm) {}

  public default void setFlywheelVoltage(Voltage voltage) {}

  public default void setFeederVoltage(Voltage voltage) {}

  public default void setFeederSpeed(AngularVelocity speed) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
