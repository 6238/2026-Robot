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
  public static final CANBus CAN_BUS = new CANBus("canivore");

  // When true, flywheel spins up automatically during active hub shifts (and 2s before each starts)
  public static final boolean SPINUP_WHEN_HUB_ACTIVE = true;

  public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
  public static final Distance SHOOTER_HOOD_HEIGHT = Inches.of(28.0);

  public static final double FEEDER_GEARING = 18.0 / 12.0;
  public static final double FLYWHEEL_GEARING = 18.0 / 24.0;

  public static final LoggedNetworkPIDFeedforwardGains FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.6, // kP
          0, // kI
          0, // kD
          0.172 * 1 / FLYWHEEL_GEARING, // kA
          0.0631 * 1 / FLYWHEEL_GEARING, // kV
          0.25 * 1 / FLYWHEEL_GEARING, // 0.3, // kS
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
          0.6, // kP
          0.0, // kI
          0, // kD
          0.172 * 1 / FLYWHEEL_GEARING, // kA
          0.0831 * 1 / FLYWHEEL_GEARING, // kV
          0.35 * 1 / FLYWHEEL_GEARING, // 0.3, // kS
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

  public static final LoggedNetworkNumber SPINUP_FLYWHEEL_SPEED =
      new LoggedNetworkNumber("Shooter/SpinupFlywheelRPM", 50);
  public static final LoggedNetworkNumber FEEDER_SPEED =
      new LoggedNetworkNumber("Shooter/FeederRPS", 50);

  // SHOT SETPOINTS
  public static final Angle FIXED_HOOD_ANGLE_DEGREES = Degrees.of(60.5);
  // Fraction below target at which bang-through (full 12 V) is applied instead of PID/FF
  public static final double FLYWHEEL_BANG_THROUGH_THRESHOLD = 0.04;
  // RPS below target at which recovery acceleration is injected
  public static final double FLYWHEEL_RECOVERY_THRESHOLD_RPS = 3.5;
  // Acceleration (RPS/s) injected during flywheel speed recovery
  // Set just above measured physical max (~110 RPS/s from 0→55 RPS in 0.5s)
  public static final double FLYWHEEL_RECOVERY_ACCELERATION_RPS2 = 200.0;
  public static final AngularVelocity FLYWHEEL_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(0.5);
  public static final AngularVelocity FEEDER_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(2.0);
  public static final LoggedNetworkNumber FEEDER_REVERSE_VOLTAGE =
      new LoggedNetworkNumber("Shooter/FeederReverseVolts", 8.0);
  public static final AngularVelocity BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT =
      RotationsPerSecond.of(0.8);
  public static final Distance HUB_POSITION_TOLERANCE = Meters.of(0.04);
  public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(1.25);
  public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(1.25);
  public static final double MIN_SHOT_DISTANCE_METERS = 2.2;
  public static final double HUB_NEAR_DISTANCE_METERS = 3.0;
  public static final double HUB_HIGH_ROBOT_SPEED_MPS = 2.5;

  // Ball speed physics:
  //   ball_speed_mps = flywheel_rps × (2π × wheel_radius) / ENERGY_TRANSFER_COEFF
  //   horizontal_speed = ball_speed × cos(hood_angle)
  //   flight_time = distance / horizontal_speed
  public static final double FLYWHEEL_ENERGY_TRANSFER_COEFF = 1.5;
  public static final double BALL_SPEED_PER_FLYWHEEL_RPS =
      2 * Math.PI * SHOOTER_WHEEL_RADIUS.in(Meters) / FLYWHEEL_ENERGY_TRANSFER_COEFF;

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
    FLYWHEEL_MAP.put(2.32, 43.9);
    FLYWHEEL_MAP.put(2.53, 45.0);
    FLYWHEEL_MAP.put(3.52, 52.5);
    FLYWHEEL_MAP.put(4.14, 55.9);

    // Physics-based: t_lead = d / (rps × BALL_SPEED_PER_FLYWHEEL_RPS × cos(hood))
    LEAD_TIME_MAP = new ExtrapolatingDoubleTreeMap();
    double cosHood = Math.cos(FIXED_HOOD_ANGLE_DEGREES.in(Radians));
    for (double d = 0.5; d <= 8.0; d += 0.25) {
      double rps = FLYWHEEL_MAP.get(d);
      double tFlight = d / (rps * BALL_SPEED_PER_FLYWHEEL_RPS * cosHood);
      LEAD_TIME_MAP.put(d, tFlight);
    }

    PASSING_FLYWHEEL_MAP = new ExtrapolatingDoubleTreeMap();
    PASSING_FLYWHEEL_MAP.put(1.5, 27.5 / 0.75);
    PASSING_FLYWHEEL_MAP.put(3.0, 34.75 / 0.75);

    PASSING_LEAD_TIME_MAP = new ExtrapolatingDoubleTreeMap();
    for (double d = 0.5; d <= 8.0; d += 0.25) {
      double rps = PASSING_FLYWHEEL_MAP.get(d);
      double tFlight = d / (rps * BALL_SPEED_PER_FLYWHEEL_RPS * cosHood);
      PASSING_LEAD_TIME_MAP.put(d, tFlight);
    }
  }
}
