package frc.robot.subsystems.climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

public class ClimberConstants {
    public static final CANBus CAN_BUS = CANBus.roboRIO();
    public static final int CLIMBER_MOTOR_ID = 0;

    public static final double CLIMBER_GEARING = 0.0;
    public static final InvertedValue CLIMBER_MOTOR_DIRECTION = 
            InvertedValue.CounterClockwise_Positive;
}
