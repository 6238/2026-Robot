package frc.robot.generated;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;

/** Instance wrapper so code can do RobotIdentity.getTunerConstants()..FrontLeft, etc. */
public final class RobotTunerConstants {
  public final LinearVelocity kSpeedAt12Volts;

  public final SwerveDrivetrainConstants DrivetrainConstants;

  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  public RobotTunerConstants(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontRight,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backRight,
      LinearVelocity speedAt12Volts) {
    this.DrivetrainConstants = drivetrainConstants;
    this.FrontLeft = frontLeft;
    this.FrontRight = frontRight;
    this.BackLeft = backLeft;
    this.BackRight = backRight;
    this.kSpeedAt12Volts = speedAt12Volts;
  }

  /** Convenience if you commonly pass these as varargs. */
  public SwerveModuleConstants<?, ?, ?>[] Modules() {
    return new SwerveModuleConstants<?, ?, ?>[] {FrontLeft, FrontRight, BackLeft, BackRight};
  }
}
