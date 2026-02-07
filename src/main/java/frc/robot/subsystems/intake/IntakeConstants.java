package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeConstants {
  public static final CANBus CAN_BUS = CANBus.roboRIO();
  public static final int INTAKE_MOTOR_ID = 55;
  public static final int INTAKE_ARM_MOTOR_ID = 54;

  public static final double INTAKE_GEARING = 1.0;
  public static final InvertedValue INTAKE_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;

  public static final double INTAKE_ARM_GEARING = 15 * (72 / 24);
  public static final InvertedValue INTAKE_ARM_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;
  public static final Angle INTAKE_START_VALUE = Degrees.of(0.0);
  public static final LoggedNetworkPIDFeedforwardGains INTAKE_ARM_GAINS =
      new LoggedNetworkPIDFeedforwardGains(3.5, 0.07, 0, 0.19, 5, 0, 0.34, "Intake_ARM");
  public static final MotionMagicConfigs INTAKE_ARM_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2));

  public static final LoggedNetworkNumber INTAKE_VOLTAGE =
      new LoggedNetworkNumber("Intake/IntakeVoltage", 5);
}
