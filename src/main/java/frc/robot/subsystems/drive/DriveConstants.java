package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class DriveConstants {
  /** Calculated from Log */
  public static final AngularVelocity MAX_MODULE_ANGULAR_VELOCITY = RadiansPerSecond.of(15);

  /**
   * When true, skips the SwerveSetpointGenerator and sends ChassisSpeeds directly to modules via
   * kinematics. Eliminates setpoint lag entirely at the cost of kinematic constraint enforcement.
   * Affects ALL drive paths including auto. For testing only; leave false for competition.
   */
  public static boolean BYPASS_SETPOINT_GENERATOR = false;
}
