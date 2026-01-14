package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
  public static final int SHOOTER_MOTOR_ID = 40;
  public static final int FEEDER_MOTOR_ID = 38;

  public static final CANBus CAN_BUS = CANBus.roboRIO();

  public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
  public static final Distance SHOOTER_HOOD_HEIGHT = Inches.of(28.0);
}
