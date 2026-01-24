package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeConstants {
  public static final CANBus CAN_BUS = CANBus.roboRIO();
  public static final int INTAKE_MOTOR_ID = 46;

  public static final double INTAKE_GEARING = 1.0;
  public static final InvertedValue INTAKE_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;

  public static final LoggedNetworkNumber INTAKE_VOLTAGE =
      new LoggedNetworkNumber("Intake/IntakeVoltage", 5);
}
