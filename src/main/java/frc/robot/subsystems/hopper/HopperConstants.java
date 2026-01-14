package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.CANBus;

public class HopperConstants {
    public static final CANBus CAN_BUS = CANBus.roboRIO();
    public static final int INDEXER_MOTOR_ID = 40;

    public static final double INDEXER_GEARING = 1.0;
}
