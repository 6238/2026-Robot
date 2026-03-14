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

  public static final double INTAKE_GEARING = 34 / 12 * 18 / 34;
  public static final InvertedValue INTAKE_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;
  public static final LoggedNetworkPIDFeedforwardGains INTAKE_GAINS =
      new LoggedNetworkPIDFeedforwardGains(0.03, 0, 0.00, 0, 0.17, 0, 0, "intake");

  public static final double INTAKE_ARM_GEARING = 25 * 2;
  public static final InvertedValue INTAKE_ARM_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;
  public static final Angle INTAKE_START_VALUE = Degrees.of(120.0);
  public static final LoggedNetworkNumber INTAKE_DOWN_VALUE =
      new LoggedNetworkNumber("INTAKE_DOWN_ANGLE", -40.0);
  public static final LoggedNetworkNumber INTAKE_UP_VALUE =
      new LoggedNetworkNumber("INTAKE_UP_ANGLE", 70.0);
  public static final LoggedNetworkPIDFeedforwardGains INTAKE_ARM_GAINS =
      new LoggedNetworkPIDFeedforwardGains(2.6, 0.03, 0, 0.30, 5.44, 0, 0.3, "Intake_ARM");
  public static final MotionMagicConfigs INTAKE_ARM_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2));

  public static final LoggedNetworkNumber INTAKE_SPEED =
      new LoggedNetworkNumber("Intake/INTAKE_SPEED", 70);

  // Jam prevention
  public static final double STALL_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double STALL_VELOCITY_THRESHOLD_RPS = 8.0;
  public static final double STALL_DEBOUNCE_SECONDS = 0.25;
  public static final double JAM_REVERSE_MIN_DURATION_SECONDS = 0.1875;
}
