package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HopperConstants {
  public static final CANBus CAN_BUS = CANBus.roboRIO();
  public static final int INDEXER_MOTOR_ID = 40;
  public static final int TOP_INDEXER_MOTOR_ID = 41;

  public static final double INDEXER_GEARING = 1.0;
  public static final InvertedValue INDEXER_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

  public static final double TOP_INDEXER_GEARING = 1.0;
  public static final InvertedValue TOP_INDEXER_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

  public static final LoggedNetworkNumber INDEXER_VOLTAGE =
      new LoggedNetworkNumber("Hopper/IndexerVoltage", 5);

  public static final LoggedNetworkNumber REVERSE_INDEXER_VOLTAGE =
      new LoggedNetworkNumber("Hopper/IndexerVoltage", -5);

  public static final LoggedNetworkNumber TOP_INDEXER_VOLTAGE =
      new LoggedNetworkNumber("Hopper/TopIndexerVoltage", 5);

  public static final LoggedNetworkNumber REVERSE_TOP_INDEXER_VOLTAGE =
      new LoggedNetworkNumber("Hopper/TopIndexerVoltage", -5);
}
