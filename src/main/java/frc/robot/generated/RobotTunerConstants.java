package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/** Instance wrapper so code can do RobotIdentity.getTunerConstants()..FrontLeft, etc. */
public final class RobotTunerConstants {
  public final SwerveDrivetrainConstants DrivetrainConstants;

  public final SwerveModuleConstants<?, ?, ?> FrontLeft;
  public final SwerveModuleConstants<?, ?, ?> FrontRight;
  public final SwerveModuleConstants<?, ?, ?> BackLeft;
  public final SwerveModuleConstants<?, ?, ?> BackRight;

  public RobotTunerConstants(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?> frontLeft,
      SwerveModuleConstants<?, ?, ?> frontRight,
      SwerveModuleConstants<?, ?, ?> backLeft,
      SwerveModuleConstants<?, ?, ?> backRight) {
    this.DrivetrainConstants = drivetrainConstants;
    this.FrontLeft = frontLeft;
    this.FrontRight = frontRight;
    this.BackLeft = backLeft;
    this.BackRight = backRight;
  }

  /** Convenience if you commonly pass these as varargs. */
  public SwerveModuleConstants<?, ?, ?>[] Modules() {
    return new SwerveModuleConstants<?, ?, ?>[] {FrontLeft, FrontRight, BackLeft, BackRight};
  }
}
