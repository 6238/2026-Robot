package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;

public class ClimberConstants {
  public static final CANBus CAN_BUS = CANBus.roboRIO();
  public static final int CLIMBER_MOTOR_ID = 0;

  // These must all be the same, just with PerSecond added to unit for each derivative
  public static final DistanceUnit DISTANCE_GEARING_UNIT = Inches;
  public static final LinearVelocityUnit LINEAR_VELOCITY_GEARING_UNIT = InchesPerSecond;
  public static final LinearAccelerationUnit LINEAR_ACCELERATION_GEARING_UNIT = InchesPerSecondPerSecond;

  public static final double CLIMBER_GEARING = 0.0; // Convert Rotations to distance unit
  public static final InvertedValue CLIMBER_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;

  public static final LoggedNetworkPIDFeedforwardGains CLIMBER_GAINS =
      new LoggedNetworkPIDFeedforwardGains(0, 0, 0, 0, 0, 0, 0, "Climber");

  public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(0).withMotionMagicAcceleration(0);
}
