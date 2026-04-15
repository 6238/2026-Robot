package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class DriveConstants {
  /** Max steer angular velocity passed to SwerveSetpointGenerator. Raised from 30 to reduce
   * module reorientation lag (~100ms → ~31ms for 90° turn). */
  public static final AngularVelocity MAX_MODULE_ANGULAR_VELOCITY = RadiansPerSecond.of(50);

  /** When true, skips the SwerveSetpointGenerator and sends ChassisSpeeds directly to modules.
   * Eliminates setpoint lag entirely; use for testing. Auto path always uses the generator. */
  public static boolean BYPASS_SETPOINT_GENERATOR = false;
}
