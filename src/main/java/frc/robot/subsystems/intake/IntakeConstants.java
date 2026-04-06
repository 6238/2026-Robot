package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.LoggedNetworkPIDFeedforwardGains;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeConstants {
  public static final CANBus ROLLER_CAN_BUS = CANBus.roboRIO();
  public static final CANBus PIVOT_CAN_BUS = new CANBus("canivore");
  public static final int INTAKE_MOTOR_ID = 59;
  public static final int INTAKE_FOLLOWER_MOTOR_ID = 55;
  public static final int INTAKE_ARM_MOTOR_ID = 54;

  public static final double INTAKE_GEARING = 1.5;
  public static final InvertedValue INTAKE_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;
  public static final LoggedNetworkPIDFeedforwardGains INTAKE_GAINS =
      new LoggedNetworkPIDFeedforwardGains(0.25, 0, 0.00, 0.15, 0.17, 0, 0.2, "intake");

  public static final double INTAKE_ARM_GEARING = 35 * 2;
  public static final InvertedValue INTAKE_ARM_MOTOR_DIRECTION =
      InvertedValue.CounterClockwise_Positive;
  public static final Angle INTAKE_START_VALUE = Degrees.of(110.0);
  public static final LoggedNetworkNumber INTAKE_DOWN_VALUE =
      new LoggedNetworkNumber("INTAKE_DOWN_ANGLE", -10);
  public static final LoggedNetworkNumber INTAKE_UP_VALUE =
      new LoggedNetworkNumber("INTAKE_UP_ANGLE", 60.0);
  public static final LoggedNetworkPIDFeedforwardGains INTAKE_ARM_GAINS =
      new LoggedNetworkPIDFeedforwardGains(48, 1, 5, 0.17, 7.8, 0, 0.3, "Intake_ARM");
  public static final MotionMagicConfigs INTAKE_ARM_MOTION_MAGIC_CONFIGS =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1.5))
          .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(3));

  public static final LoggedNetworkNumber INTAKE_SPEED =
      new LoggedNetworkNumber("Intake/INTAKE_SPEED", 70);

  // Pivot backlash preload (run into hard stop before auto)
  public static final double PIVOT_PRELOAD_VOLTAGE_VOLTS = 2.0;
  public static final double PIVOT_PRELOAD_CURRENT_THRESHOLD_AMPS = 10.0;
  public static final double PIVOT_PRELOAD_TIMEOUT_SECONDS = 1.0;

  // Jam prevention
  public static final double STALL_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double STALL_VELOCITY_THRESHOLD_RPS = 8.0;
  public static final double STALL_DEBOUNCE_SECONDS = 0.25;
  public static final double JAM_REVERSE_MIN_DURATION_SECONDS = 0.1875;

  // Pivot crawl-up (slow upward creep with current-spike backoff)
  public static final LoggedNetworkNumber CRAWL_UP_VOLTAGE_VOLTS =
      new LoggedNetworkNumber("Intake/CrawlUpVoltage", 3.5);
  public static final LoggedNetworkNumber CRAWL_BACKOFF_VOLTAGE_VOLTS =
      new LoggedNetworkNumber("Intake/CrawlBackoffVoltage", 1.5);
  public static final LoggedNetworkNumber CRAWL_CURRENT_THRESHOLD_AMPS =
      new LoggedNetworkNumber("Intake/CrawlCurrentThreshold", 15.0);
  public static final LoggedNetworkNumber CRAWL_BACKOFF_DURATION_SECONDS =
      new LoggedNetworkNumber("Intake/CrawlBackoffDuration", 0.2);
  public static final LoggedNetworkNumber CRAWL_UP_VOLTAGE_MIN_VOLTS =
      new LoggedNetworkNumber("Intake/CrawlUpVoltageMin", 0.5);
  public static final LoggedNetworkNumber CRAWL_CURRENT_THRESHOLD_MIN_AMPS =
      new LoggedNetworkNumber("Intake/CrawlCurrentThresholdMin", 3.0);
  public static final double CRAWL_MAX_OFFSET_DEGREES = 10.0;

  // Pivot oscillation during shooting
  // Center sweeps from (INTAKE_DOWN_VALUE + OSCILLATE_CENTER_OFFSET) to
  // (INTAKE_UP_VALUE - OSCILLATE_CENTER_OFFSET) at OSCILLATE_SWEEP_RATE_DPS degrees/sec,
  // then the setpoint sinusoids around that center.
  public static final double OSCILLATE_CENTER_OFFSET_DEGREES = 20.0;
  public static final LoggedNetworkNumber OSCILLATE_AMPLITUDE_DEGREES =
      new LoggedNetworkNumber("Intake/OscillateAmplitude", 10.0);
  public static final LoggedNetworkNumber OSCILLATE_FREQUENCY_HZ =
      new LoggedNetworkNumber("Intake/OscillateFrequency", 4.0);
  public static final LoggedNetworkNumber OSCILLATE_SWEEP_RATE_DPS =
      new LoggedNetworkNumber("Intake/OscillateSweepRate", 15.0);

  // DRS (Dynamic Recovery System) — intake lower block detection & mitigation
  // Detection: pivot targeting DOWN, high current, AND stuck above target for DRS_STUCK_TIMEOUT_S
  public static final LoggedNetworkNumber DRS_CURRENT_THRESHOLD_AMPS =
      new LoggedNetworkNumber("DRS/CurrentThresholdAmps", 15.0);
  public static final LoggedNetworkNumber DRS_ANGLE_ERROR_THRESHOLD_DEGREES =
      new LoggedNetworkNumber("DRS/AngleErrorThresholdDeg", 30.0);
  public static final double DRS_TARGET_TOLERANCE_DEGREES = 5.0;
  public static final double DRS_STUCK_TIMEOUT_SECONDS = 0.1;
  // Mitigation: lift to middle, drive backward, lower back down
  public static final LoggedNetworkNumber DRS_MIDDLE_ANGLE_DEGREES =
      new LoggedNetworkNumber("DRS/MiddleAngleDeg", 70.0);
  public static final double DRS_MIDDLE_ANGLE_TOLERANCE_DEGREES = 5.0;
  public static final double DRS_DOWN_ANGLE_TOLERANCE_DEGREES = 5.0;
  public static final LoggedNetworkNumber DRS_BACKUP_SPEED_MPS =
      new LoggedNetworkNumber("DRS/BackupSpeedMps", 1.0);
  public static final double DRS_BACKUP_DURATION_SECONDS = 0.3;

  // Roller recovery acceleration
  public static final double ROLLER_RECOVERY_THRESHOLD_RPS = 2.0;
  public static final double ROLLER_RECOVERY_ACCELERATION_RPS2 = 150.0;

  // Slingshot (roller pullback + release during shooting)
  public static final double SLINGSHOT_PULLBACK_ROTATIONS = 0.5;
  public static final LoggedNetworkNumber SLINGSHOT_PULLBACK_SPEED_RPS =
      new LoggedNetworkNumber("Intake/SlingshotPullbackSpeed", 5.0);
  public static final LoggedNetworkNumber SLINGSHOT_FORWARD_WAIT_SECONDS =
      new LoggedNetworkNumber("Intake/SlingshotForwardWait", 0.5);
}
