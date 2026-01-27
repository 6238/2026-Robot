package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShotPlanner {
  public static ShotSetpoint createShotSetpoint(
      Pose2d drivePose, ChassisSpeeds driveChassisSpeeds) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            driveChassisSpeeds.vxMetersPerSecond,
            driveChassisSpeeds.vyMetersPerSecond,
            driveChassisSpeeds.omegaRadiansPerSecond,
            drivePose.getRotation());

    Translation2d hubTranslation2d = Constants.HUB_POSE_3D.getTranslation().toTranslation2d();
    double distanceCurrently = drivePose.getTranslation().getDistance(hubTranslation2d);
    double leadTimeSec =
        ShooterConstants.LEAD_TIME_DIST_OFFSET
            + ShooterConstants.LEAD_TIME_DIST_SLOPE * distanceCurrently;
    Translation2d predictedTranslation =
        hubTranslation2d.plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * -leadTimeSec,
                fieldSpeeds.vyMetersPerSecond * -leadTimeSec));

    Logger.recordOutput(
        "ShotPlanner/predictedTranslation", new Pose2d(predictedTranslation, Rotation2d.kZero));
    Logger.recordOutput("ShotPlanner/leadTime", leadTimeSec);

    double predictedDistanceM = drivePose.getTranslation().getDistance(predictedTranslation);

    double flywheelSpeed =
        ShooterConstants.FLYWHEEL_DIST_OFFSET
            + ShooterConstants.FLYWHEEL_DIST_SLOPE * predictedDistanceM;

    Pose2d lookAtHubPose2d =
        new Pose2d(
            drivePose.getTranslation(),
            predictedTranslation.minus(drivePose.getTranslation()).getAngle());

    return new ShotSetpoint(
        RotationsPerSecond.of(flywheelSpeed),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        lookAtHubPose2d,
        driveChassisSpeeds,
        predictedTranslation);
  }

  public static ShotSetpoint createPassSetpoint(
      Pose2d targetPassPoint, Pose2d drivePose, ChassisSpeeds driveChassisSpeeds) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            driveChassisSpeeds.vxMetersPerSecond,
            driveChassisSpeeds.vyMetersPerSecond,
            driveChassisSpeeds.omegaRadiansPerSecond,
            drivePose.getRotation());

    Translation2d targetTranslation2d = targetPassPoint.getTranslation();
    double distanceCurrently = drivePose.getTranslation().getDistance(targetTranslation2d);
    double leadTimeSec =
        ShooterConstants.PASSING_FLYWHEEL_DIST_OFFSET
            + ShooterConstants.PASSING_LEAD_TIME_DIST_SLOPE * distanceCurrently;
    Translation2d predictedTranslation =
        targetTranslation2d.plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * -leadTimeSec,
                fieldSpeeds.vyMetersPerSecond * -leadTimeSec));

    Logger.recordOutput(
        "ShotPlanner/predictedTranslation", new Pose2d(predictedTranslation, Rotation2d.kZero));
    Logger.recordOutput("ShotPlanner/leadTime", leadTimeSec);

    double predictedDistanceM = drivePose.getTranslation().getDistance(predictedTranslation);

    double flywheelSpeed =
        ShooterConstants.PASSING_FLYWHEEL_DIST_OFFSET
            + ShooterConstants.PASSING_FLYWHEEL_DIST_SLOPE * predictedDistanceM;

    Pose2d lookAtHubPose2d =
        new Pose2d(
            drivePose.getTranslation(),
            predictedTranslation.minus(drivePose.getTranslation()).getAngle());

    return new ShotSetpoint(
        RotationsPerSecond.of(flywheelSpeed),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        lookAtHubPose2d,
        driveChassisSpeeds,
        predictedTranslation);
  }

  public static class ShotSetpoint {
    public AngularVelocity flywheelSpeed;
    public Angle hoodAngle;
    public Pose2d robotPose;
    public Translation2d target;
    public ChassisSpeeds robotSpeeds;

    public ShotSetpoint(
        AngularVelocity flywheelSpeed,
        Angle hoodAngle,
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        Translation2d target) {
      this.flywheelSpeed = flywheelSpeed;
      this.hoodAngle = hoodAngle;
      this.robotPose = robotPose;
      this.robotSpeeds = robotSpeeds;
      this.target = target;
    }

    public ShotSetpoint() {
      this.flywheelSpeed = RotationsPerSecond.of(0);
      this.hoodAngle = Degrees.of(0);
      this.robotPose = new Pose2d();
      this.robotSpeeds = new ChassisSpeeds();
      this.target = new Translation2d();
    }
  }
}
