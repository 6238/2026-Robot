package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;

public class ShooterConstants {
  public static final int FLYWHEEL_MOTOR_ID = 40;
  public static final int FEEDER_MOTOR_ID = 38;

  public static final CANBus CAN_BUS = CANBus.roboRIO();

  public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
  public static final Distance SHOOTER_HOOD_HEIGHT = Inches.of(28.0);
  
  public static final double FEEDER_GEARING = 1;
  public static final double FLYWHEEL_GEARING = 1;

  public static final LoggedNetworkPIDFeedforwardGains FLYWHEEL_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.1, // kP
          0.0, // kI
          0.0, // kD
          0.0, // kA
          0.1, // kV
          0.0, // kS
          0.0, // kG
          "ShooterFlywheel"
      );
}
