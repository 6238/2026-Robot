package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterSetpoint {
  public AngularVelocity flywheelSpeed;
  public Angle hoodAngle;
  public Pose2d robotPose;
  public Translation2d target;
  public ChassisSpeeds robotSpeeds;

  public ShooterSetpoint(
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

  //   public static ShooterSetpoint generateNearestShooterSetpoint(
  //       Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
  //     double flywheelSpeed =
  //         ShooterConstants.FLYWHEEL_DIST_OFFSET
  //             + ShooterConstants.FLYWHEEL_DIST_SLOPE
  //                 * robotPose
  //                     .getTranslation()
  //                     .getDistance(Constants.HUB_POSE_3D.getTranslation().toTranslation2d());
  //     Logger.recordOutput(
  //         "dist",
  //         robotPose
  //             .getTranslation()
  //             .getDistance(Constants.HUB_POSE_3D.getTranslation().toTranslation2d()));

  //   Pose2d lookAtHubPose2d =
  //       new Pose2d(
  //           robotPose.getTranslation(),
  //           Constants.HUB_POSE_3D
  //               .getTranslation()
  //               .toTranslation2d()
  //               .minus(robotPose.getTranslation())
  //               .getAngle());

  //   return new ShooterSetpoint(
  //       RotationsPerSecond.of(flywheelSpeed),
  //       ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
  //       lookAtHubPose2d,
  //       new ChassisSpeeds());
  // }

  public static ShooterSetpoint generateNearestShooterSetpoint(
      Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            robotPose.getRotation());

    Translation2d hubTranslation2d = Constants.HUB_POSE_3D.getTranslation().toTranslation2d();
    double distanceCurrently = robotPose.getTranslation().getDistance(hubTranslation2d);
    double leadTimeSec =
        ShooterConstants.LEAD_TIME_DIST_OFFSET
            + ShooterConstants.LEAD_TIME_DIST_SLOPE * distanceCurrently;

    Translation2d predictedTranslation =
        hubTranslation2d.plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * -leadTimeSec,
                fieldSpeeds.vyMetersPerSecond * -leadTimeSec));

    Logger.recordOutput("predictedTranslation", new Pose2d(predictedTranslation, Rotation2d.kZero));

    Logger.recordOutput("leadTime", leadTimeSec);

    double predictedDistanceM = robotPose.getTranslation().getDistance(predictedTranslation);

    double flywheelSpeed =
        ShooterConstants.FLYWHEEL_DIST_OFFSET
            + ShooterConstants.FLYWHEEL_DIST_SLOPE * predictedDistanceM;

    Pose2d lookAtHubPose2d =
        new Pose2d(
            robotPose.getTranslation(),
            predictedTranslation.minus(robotPose.getTranslation()).getAngle());

    return new ShooterSetpoint(
        RotationsPerSecond.of(flywheelSpeed),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        lookAtHubPose2d,
        robotRelativeSpeeds,
        predictedTranslation);
  }
}
