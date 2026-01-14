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
    public boolean shooterTalonConnected = false;
    public boolean feederTalonConnected = false;

    public AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    public AngularAcceleration shooterAcceleration = RotationsPerSecondPerSecond.of(0);
    public Current shooterAppliedCurrent = Amps.of(0);
    public Voltage shooterAppliedVoltage = Volts.of(0);

    public AngularVelocity feederVelocity = RotationsPerSecond.of(0);
    public AngularAcceleration fAcceleration = RotationsPerSecondPerSecond.of(0);
    public Current feederAppliedCurrent = Amps.of(0);
    public Voltage feederAppliedVoltage = Volts.of(0);
  }

  public default void setShooterRPM(AngularVelocity rpm) {}

  public default void setShooterVoltage(Voltage voltage) {}

  public default void setFeederVoltage(Voltage voltage) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
