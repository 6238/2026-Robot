package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeTalonConnected = false;
    public boolean intakeArmTalonConnected = false;

    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration intakeAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public Current intakeAppliedCurrent = Amps.of(0.0);
    public Voltage intakeAppliedVoltage = Volts.of(0.0);

    public Angle intakeArmPosition = Degrees.of(0.0);
    public AngularVelocity intakeArmVelocity = RotationsPerSecond.of(0.0);
    public AngularAcceleration intakeArmAcceleration = RotationsPerSecondPerSecond.of(0.0);
    public Current intakeArmAppliedCurrent = Amps.of(0.0);
    public Voltage intakeArmAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
  ;

  public default void setIntakeVoltage(Voltage voltage) {}
  ;

  public default void setIntakePosition(Angle targetAngle) {}
  ;

  public default void setIntakeVelocity(AngularVelocity speed) {}

  public default void resetArmAngle() {}
  ;
}
