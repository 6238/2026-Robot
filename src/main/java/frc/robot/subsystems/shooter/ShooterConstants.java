package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterConstants {
  public static final int FLYWHEEL_MOTOR_ID = 40;
  public static final int FEEDER_MOTOR_ID = 38;

  public static final InvertedValue FLYWHEEL_INVERTED = InvertedValue.Clockwise_Positive;
  public static final InvertedValue FEEDER_INVERTED = InvertedValue.Clockwise_Positive;

  public static final CANBus CAN_BUS = CANBus.roboRIO();

  public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
  public static final Distance SHOOTER_HOOD_HEIGHT = Inches.of(28.0);

  public static final double FEEDER_GEARING = 1;
  public static final double FLYWHEEL_GEARING = 1;

  public static final LoggedNetworkPIDFeedforwardGains FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.1, // kP
          0.04, // kI
          0.0, // kD
          0.0, // kA
          0.08, // kV
          0.13, // kS
          0.0, // kG
          "ShooterFlywheel");

  public static final MotionMagicConfigs FLYWHEEL_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs().withMotionMagicCruiseVelocity(20).withMotionMagicAcceleration(40);

  public static final LoggedNetworkNumber SPINUP_FLYWHEEL_SPEED =
      new LoggedNetworkNumber("Shooter/SpinupFlywheelRPM", 70);
  public static final LoggedNetworkNumber FEEDER_VOLTAGE =
      new LoggedNetworkNumber("Shooter/FeederVoltage", 8);
  public static final LoggedNetworkNumber LEAD_TIME_SEC =
      new LoggedNetworkNumber("Shooter/LEAD_TIME", 0.1);

  // SHOT SETPOINTS
  public static final Angle FIXED_HOOD_ANGLE_DEGREES = Degrees.of(60.5);
  public static final AngularVelocity FLYWHEEL_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(4.0);
  public static final Distance HUB_POSITION_TOLERANCE = Meters.of(0.05);
  public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(4.0);

  public static final InterpolatingTreeMap<Distance, AngularVelocity> SHOOTER_LOOKUP_TABLE =
      new InterpolatingTreeMap<Distance, AngularVelocity>(
          (Distance start, Distance end, Distance q) -> {
            double s = start.in(Meters);
            double e = end.in(Meters);
            double x = q.in(Meters);

            if (e <= s) return 0.0; // avoid divide-by-zero / bad ordering
            double t = (x - s) / (e - s);
            return Math.max(0.0, Math.min(1.0, t)); // clamp to [0, 1]
          },
          // Interpolator<AngularVelocity>: blend angular velocities
          (AngularVelocity start, AngularVelocity end, double t) -> {
            double a = start.in(RadiansPerSecond);
            double b = end.in(RadiansPerSecond);
            return RadiansPerSecond.of(a + (b - a) * t);
          });

  static {
    SHOOTER_LOOKUP_TABLE.put(Meters.of(2.679), RotationsPerSecond.of(90.0));
  }

  public static final InterpolatingDoubleTreeMap SHOOTER_LEAD_TIME_FROM_DIST =
      new InterpolatingDoubleTreeMap();

  static {
    SHOOTER_LEAD_TIME_FROM_DIST.put(1.0, 1.0);
  }
}
