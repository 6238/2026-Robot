package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean intakeTalonConnected = false;

    public Angle intakePosition = Rotations.of(0.0);
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration intakeAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public Current intakeSupplyCurrent = Amps.of(0.0);
    public Voltage intakeAppliedVoltage = Volts.of(0.0);
    public Temperature intakeTemperature = Celsius.of(0.0);
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setIntakeVoltage(Voltage voltage) {}

  public default void setIntakeVelocity(AngularVelocity speed) {}
}
