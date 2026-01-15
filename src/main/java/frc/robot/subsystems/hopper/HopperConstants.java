package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.util.TestMode.MotorTestProfile;

public class HopperConstants {
    public static final CANBus CAN_BUS = CANBus.roboRIO();
    public static final int INDEXER_MOTOR_ID = 40;

    public static final double INDEXER_GEARING = 1.0;
    public static final InvertedValue INDEXER_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

    public static final LoggedNetworkNumber INDEXER_VOLTAGE =
        new LoggedNetworkNumber("Hopper/IndexerVoltage", 5);

    public static final LoggedNetworkNumber REVERSE_INDEXER_VOLTAGE =
        new LoggedNetworkNumber("Hopper/IndexerVoltage", -5);

    
    public static final MotorTestProfile HOPPER_TEST_PROFILE = new MotorTestProfile(
        "Hopper Indexer",
        Volts.of(INDEXER_VOLTAGE.get()),
        Seconds.of(3.0),
        Seconds.of(0.5),
        RotationsPerSecond.of(0.0),
        Amps.of(-1),
        Amps.of(0.15)
    );
}
