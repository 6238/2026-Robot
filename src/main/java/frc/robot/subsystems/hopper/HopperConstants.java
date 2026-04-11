package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HopperConstants {
  public static final boolean USE_TOP_INDEXER = true;

  public static final CANBus CAN_BUS = new CANBus("canivore");
  public static final int INDEXER_MOTOR_ID = 41;
  public static final int TOP_INDEXER_MOTOR_ID = 46;

  public static final double INDEXER_GEARING = 1.0;
  public static final InvertedValue INDEXER_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;

  public static final double TOP_INDEXER_GEARING = 1.0;
  public static final InvertedValue TOP_INDEXER_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;

  public static final AngularVelocity HOPPER_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(10);
  public static final AngularVelocity TOP_INDEXER_JAM_THRESHOLD = RotationsPerSecond.of(2);

  public static final double simulatedHopperThroughput = 8;

  public static final LoggedNetworkNumber INDEXER_SPEED =
      new LoggedNetworkNumber("Hopper/IndexerSpeedRPS", 40);

  public static final LoggedNetworkNumber TOP_INDEXER_SPEED =
      new LoggedNetworkNumber("Hopper/TopIndexerSpeedRPS", 40);

  public static final LoggedNetworkPIDFeedforwardGains INDEXER_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.6, // kP
          0.0, // kI
          0.0, // kD
          0.0, // kA
          0.10, // kV
          0.24, // kS
          0.0, // kG
          "HopperIndexer");

  public static final LoggedNetworkPIDFeedforwardGains TOP_INDEXER_GAINS =
      new LoggedNetworkPIDFeedforwardGains(
          0.4, // kP
          0.0, // kI
          0.0, // kD
          0.08, // kA
          0.10, // kV
          0.24, // kS
          0.0, // kG
          "HopperTopIndexer");

  public static final MotionMagicConfigs INDEXER_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs().withMotionMagicAcceleration(300);
}
