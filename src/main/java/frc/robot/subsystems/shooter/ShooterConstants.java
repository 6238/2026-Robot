package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.ExtrapolatingDoubleTreeMap;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterConstants {
  public static final int FLYWHEEL_MOTOR_ID = 39;
  public static final int FLYWHEEL2_MOTOR_ID = 48;
  public static final int FEEDER_MOTOR_ID = 40;

  public static final InvertedValue FLYWHEEL_INVERTED = InvertedValue.Clockwise_Positive;
  public static final InvertedValue FEEDER_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final CANBus CAN_BUS = CANBus.roboRIO();

  public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
  public static final Distance SHOOTER_HOOD_HEIGHT = Inches.of(28.0);

  public static final double FEEDER_GEARING = 18.0 / 12.0;
  public static final double FLYWHEEL_GEARING = 18.0 / 24.0;

  public static final LoggedNetworkPIDFeedforwardGains FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.0 * 0.75, // kP
          0.0, // kI
          0, // kD
          0.02, // kA
          0.085, // kV
          0.3, // 0.3, // kS
          0.0, // kG
          "ShooterFlywheel");

  public static final LoggedNetworkPIDFeedforwardGains SIM_FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.1, // kP
          0.0, // kI
          0.08, // kD
          0.03, // kA
          0.085, // kV
          0.3, // kS // 6000rpm 100rps 6v
          0.0, // kG
          "ShooterFlywheel");

  public static final MotionMagicConfigs FLYWHEEL_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs().withMotionMagicAcceleration(300);

  public static final LoggedNetworkPIDFeedforwardGains FEEDER_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.04 * 1.5, // kP
          0.0, // kI
          0, // kD
          1.82 * 1.5, // kA
          0.12 * 1.5, // kV
          0.1 * 1.5, // kS
          0.0, // kG
          "ShooterFeeder");

  public static final LoggedNetworkPIDFeedforwardGains SIM_FEEDER_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.5, // kP
          0.0, // kI
          0.0, // kD
          0.0, // kA
          0.24, // kV
          0.1, // kS
          0.0, // kG
          "ShooterFeeder");

  // RPS of flywheel speed added per RPS of feeder deficit (feeder below target → boost flywheel)
  public static final LoggedNetworkNumber FEEDER_COMPENSATION_GAIN =
      new LoggedNetworkNumber("Shooter/FeederCompensationGain", 0.5);

  public static final LoggedNetworkNumber SPINUP_FLYWHEEL_SPEED =
      new LoggedNetworkNumber("Shooter/SpinupFlywheelRPM", 70);
  public static final LoggedNetworkNumber FEEDER_SPEED =
      new LoggedNetworkNumber("Shooter/FeederRPS", 60);
  public static final LoggedNetworkNumber LEAD_TIME_SEC =
      new LoggedNetworkNumber("Shooter/LEAD_TIME", 0.1);

  // SHOT SETPOINTS
  public static final Angle FIXED_HOOD_ANGLE_DEGREES = Degrees.of(60.5);
  // Fraction below target at which bang-through (full 12 V) is applied instead of PID/FF
  public static final double FLYWHEEL_BANG_THROUGH_THRESHOLD = 0.04;
  public static final AngularVelocity FLYWHEEL_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(0.5);
  public static final AngularVelocity FEEDER_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(2.0);
  public static final LoggedNetworkNumber FEEDER_REVERSE_VOLTAGE =
      new LoggedNetworkNumber("Shooter/FeederReverseVolts", 8.0);
  public static final AngularVelocity BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT =
      RotationsPerSecond.of(0.8);
  public static final Distance HUB_POSITION_TOLERANCE = Meters.of(0.04);
  public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(1.25);
  public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(1.25);
  public static final double HUB_NEAR_DISTANCE_METERS = 3.0;
  public static final double HUB_HIGH_ROBOT_SPEED_MPS = 2.5;

  // distance (m) → flywheel speed (RPS)
  public static final ExtrapolatingDoubleTreeMap FLYWHEEL_MAP;
  // distance (m) → lead time (s)
  public static final ExtrapolatingDoubleTreeMap LEAD_TIME_MAP;
  // distance (m) → passing flywheel speed (RPS)
  public static final ExtrapolatingDoubleTreeMap PASSING_FLYWHEEL_MAP;
  // distance (m) → passing lead time (s)
  public static final ExtrapolatingDoubleTreeMap PASSING_LEAD_TIME_MAP;

  static {
    FLYWHEEL_MAP = new ExtrapolatingDoubleTreeMap();
    FLYWHEEL_MAP.put(1.5, 26.2 / 0.75);
    FLYWHEEL_MAP.put(3.0, 34.0 / 0.75);

    LEAD_TIME_MAP = new ExtrapolatingDoubleTreeMap();
    LEAD_TIME_MAP.put(1.5, 0.56);
    LEAD_TIME_MAP.put(3.0, 0.77);

    PASSING_FLYWHEEL_MAP = new ExtrapolatingDoubleTreeMap();
    PASSING_FLYWHEEL_MAP.put(1.5, 27.5 * 1 / 0.75);
    PASSING_FLYWHEEL_MAP.put(3.0, 32.75 * 1 / 0.75);

    PASSING_LEAD_TIME_MAP = new ExtrapolatingDoubleTreeMap();
    PASSING_LEAD_TIME_MAP.put(1.5, 28.1);
    PASSING_LEAD_TIME_MAP.put(3.0, 28.4);
  }
}
