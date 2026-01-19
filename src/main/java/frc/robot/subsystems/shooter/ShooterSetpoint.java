package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public class ShooterSetpoint {
  public AngularVelocity flywheelSpeed;
  public Angle hoodAngle;
  public Pose2d robotPose;
  public ChassisSpeeds robotSpeeds;

  public ShooterSetpoint(
      AngularVelocity flywheelSpeed, Angle hoodAngle, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    this.flywheelSpeed = flywheelSpeed;
    this.hoodAngle = hoodAngle;
    this.robotPose = robotPose;
    this.robotSpeeds = robotSpeeds;
  }

  public static ShooterSetpoint generateNearestShooterSetpoint(Pose2d robotPose, ChassisSpeeds
  robotRelativeSpeeds) {
    double flywheelSpeed = ShooterConstants.FLYWHEEL_DIST_OFFSET + ShooterConstants.FLYWHEEL_DIST_SLOPE * robotPose.getTranslation().getDistance(Constants.HUB_POSE_3D.getTranslation().toTranslation2d());

      Pose2d lookAtHubPose2d =
              new Pose2d(
                      robotPose.getTranslation(),
  
  Constants.HUB_POSE_3D.getTranslation().toTranslation2d().minus(robotPose.getTranslation()).getAngle());

      return new ShooterSetpoint(
              RotationsPerSecond.of(flywheelSpeed), ShooterConstants.FIXED_HOOD_ANGLE_DEGREES, lookAtHubPose2d, new
  ChassisSpeeds());
  }

//   public static ShooterSetpoint generateNearestShooterSetpoint(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
//     ChassisSpeeds fieldSpeeds =
//         ChassisSpeeds.fromRobotRelativeSpeeds(
//             robotRelativeSpeeds.vxMetersPerSecond,
//             robotRelativeSpeeds.vyMetersPerSecond,
//             robotRelativeSpeeds.omegaRadiansPerSecond,
//             robotPose.getRotation());

//     Translation2d hubTranslation2d = Constants.HUB_POSE_3D.getTranslation().toTranslation2d();
//     double distanceCurrently = robotPose.getTranslation().getDistance(hubTranslation2d);
//     double leadTimeSec = ShooterConstants.LEAD_TIME_DIST_OFFSET + ShooterConstants.LEAD_TIME_DIST_SLOPE * distanceCurrently;

//     Translation2d predictedTranslation =
//         robotPose
//             .getTranslation()
//             .plus(
//                 new Translation2d(
//                     fieldSpeeds.vxMetersPerSecond * leadTimeSec,
//                     fieldSpeeds.vyMetersPerSecond * leadTimeSec));

//     Pose2d predicPose2d = new Pose2d(predictedTranslation, robotPose.getRotation());

//     double predictedDistanceM = predicPose2d.getTranslation().getDistance(hubTranslation2d);

//     double flywheelSpeed = ShooterConstants.FLYWHEEL_DIST_OFFSET + ShooterConstants.FLYWHEEL_DIST_SLOPE * predictedDistanceM;

//     Pose2d lookAtHubPose2d =
//         new Pose2d(
//             robotPose.getTranslation(), hubTranslation2d.minus(predictedTranslation).getAngle());

//     return new ShooterSetpoint(
//         RotationsPerSecond.of(flywheelSpeed),
//         ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
//         lookAtHubPose2d,
//         robotRelativeSpeeds);
//   }
}
