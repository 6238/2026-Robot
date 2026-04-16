package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean intakeArmTalonConnected = false;

    public Angle intakeArmPosition = Degrees.of(0.0);
    public AngularVelocity intakeArmVelocity = RotationsPerSecond.of(0.0);
    public Current intakeArmSupplyCurrent = Amps.of(0.0);
    public Voltage intakeArmAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(IntakePivotIOInputs inputs) {}

  public default void setIntakePosition(Angle targetAngle) {}

  public default void setIntakeArmVoltage(Voltage voltage) {}

  public default void resetArmAngle() {}

  public default void setBrakeMode(boolean brake) {}

  /** Reconfigures pivot arm current limits. No-op in sim/test. */
  public default void setDefenseMode(boolean active) {}
}
