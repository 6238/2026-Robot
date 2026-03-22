package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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

  public static final double FEEDER_GEARING = 18 / 12;
  public static final double FLYWHEEL_GEARING = 18 / 24;

  public static final LoggedNetworkPIDFeedforwardGains FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.20, // kP
          0.01, // kI
          0.03, // kD
          0.0, // kA
          0.12, // kV
          0, // kS
          0.0, // kG
          "ShooterFlywheel");

  public static final LoggedNetworkPIDFeedforwardGains SIM_FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.3, // kP
          0.0, // kI
          0.08, // kD
          0.0, // kA
          0.24, // kV
          0.1, // kS // 6000rpm 100rps 6v
          0.0, // kG
          "ShooterFlywheel");

  public static final MotionMagicConfigs FLYWHEEL_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(90).withMotionMagicAcceleration(130);

  public static final LoggedNetworkPIDFeedforwardGains FEEDER_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.7, // kP
          0.0, // kI
          0.04, // kD
          0.0, // kA
          0.123, // kV
          0.0, // kS
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
  public static final AngularVelocity FLYWHEEL_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(0.5);
  public static final AngularVelocity FEEDER_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(2.0);
  public static final LoggedNetworkNumber FEEDER_REVERSE_VOLTAGE =
      new LoggedNetworkNumber("Shooter/FeederReverseVolts", 3.0);
  public static final AngularVelocity BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT =
      RotationsPerSecond.of(0.8);
  public static final Distance HUB_POSITION_TOLERANCE = Meters.of(0.04);
  public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(1.5);
  public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(1.7);
  public static final double HUB_NEAR_DISTANCE_METERS = 3.0;
  public static final double HUB_HIGH_ROBOT_SPEED_MPS = 2.5;

  public static final double FLYWHEEL_DIST_OFFSET = 19.6;
  public static final double FLYWHEEL_DIST_SLOPE = 5.3;

  public static final double LEAD_TIME_DIST_OFFSET = 0.35; // 1.27 - 0.9125
  public static final double LEAD_TIME_DIST_SLOPE = 0.14;

  public static final double PASSING_FLYWHEEL_DIST_OFFSET = 27.8;
  public static final double PASSING_FLYWHEEL_DIST_SLOPE = 3.8;

  public static final double PASSING_LEAD_TIME_DIST_OFFSET = 0.45; // 1.27 - 0.9125
  public static final double PASSING_LEAD_TIME_DIST_SLOPE = 0.2;
}
