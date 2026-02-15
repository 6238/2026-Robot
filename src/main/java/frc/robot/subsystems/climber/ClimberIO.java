package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public boolean climberTalonConnected = false;

    public Distance climberPosition = Inches.of(0);
    public LinearVelocity climberVelocity = InchesPerSecond.of(0);
    public LinearAcceleration climberAcceleration = InchesPerSecondPerSecond.of(0);
    public Current climberCurrent = Amps.of(0);
    public Voltage climberAppliedVoltage = Volts.of(0);
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setPosition(Distance distance) {}
}
