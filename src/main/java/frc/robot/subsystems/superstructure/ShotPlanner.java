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

    Translation2d hubTranslation2d = Constants.HUB_POSE_3D.get().getTranslation().toTranslation2d();
    Translation2d robotTranslation = drivePose.getTranslation();

    // Iteratively solve for the lead time that is consistent with the distance to the virtual
    // target. Seed with distance to the real hub, then refine 3 times.
    double leadTimeSec =
        ShooterConstants.LEAD_TIME_MAP.get(robotTranslation.getDistance(hubTranslation2d));
    Translation2d virtualTarget = hubTranslation2d;
    for (int i = 0; i < 3; i++) {
      virtualTarget =
          hubTranslation2d.plus(
              new Translation2d(
                  fieldSpeeds.vxMetersPerSecond * -leadTimeSec,
                  fieldSpeeds.vyMetersPerSecond * -leadTimeSec));
      leadTimeSec = ShooterConstants.LEAD_TIME_MAP.get(robotTranslation.getDistance(virtualTarget));
    }

    double predictedDistanceM = robotTranslation.getDistance(virtualTarget);
    double flywheelSpeed = ShooterConstants.FLYWHEEL_MAP.get(predictedDistanceM);
    double feederSpeed = ShooterConstants.FEEDER_MAP.get(predictedDistanceM);

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput(
          "ShotPlanner/predictedTranslation", new Pose2d(virtualTarget, Rotation2d.kZero));
      Logger.recordOutput("ShotPlanner/leadTime", leadTimeSec);
      Logger.recordOutput("ShotPlanner/predictedDistance", predictedDistanceM);
    }

    Pose2d lookAtHubPose2d =
        new Pose2d(robotTranslation, virtualTarget.minus(robotTranslation).getAngle());

    return new ShotSetpoint(
        RotationsPerSecond.of(flywheelSpeed),
        RotationsPerSecond.of(feederSpeed),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        lookAtHubPose2d,
        driveChassisSpeeds,
        virtualTarget);
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
    Translation2d robotTranslation = drivePose.getTranslation();

    // Iteratively solve for the consistent lead time / virtual target
    double leadTimeSec =
        ShooterConstants.PASSING_LEAD_TIME_MAP.get(
            robotTranslation.getDistance(targetTranslation2d));
    Translation2d virtualTarget = targetTranslation2d;
    for (int i = 0; i < 3; i++) {
      virtualTarget =
          targetTranslation2d.plus(
              new Translation2d(
                  fieldSpeeds.vxMetersPerSecond * -leadTimeSec,
                  fieldSpeeds.vyMetersPerSecond * -leadTimeSec));
      leadTimeSec =
          ShooterConstants.PASSING_LEAD_TIME_MAP.get(robotTranslation.getDistance(virtualTarget));
    }

    double predictedDistanceM = robotTranslation.getDistance(virtualTarget);
    double flywheelSpeed =
        Math.min(
            ShooterConstants.PASSING_FLYWHEEL_MAP.get(predictedDistanceM),
            ShooterConstants.PASSING_FLYWHEEL_MAX_RPS);
    double feederSpeed = ShooterConstants.FEEDER_MAP.get(predictedDistanceM);

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput(
          "ShotPlanner/predictedTranslation", new Pose2d(virtualTarget, Rotation2d.kZero));
      Logger.recordOutput("ShotPlanner/leadTime", leadTimeSec);
      Logger.recordOutput("ShotPlanner/predictedDistance", predictedDistanceM);
    }

    Pose2d lookAtHubPose2d =
        new Pose2d(robotTranslation, virtualTarget.minus(robotTranslation).getAngle());

    return new ShotSetpoint(
        RotationsPerSecond.of(flywheelSpeed),
        RotationsPerSecond.of(feederSpeed),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        lookAtHubPose2d,
        driveChassisSpeeds,
        virtualTarget);
  }

  public static class ShotSetpoint {
    public AngularVelocity flywheelSpeed;
    public AngularVelocity feederSpeed;
    public Angle hoodAngle;
    public Pose2d robotPose;
    public Translation2d target;
    public ChassisSpeeds robotSpeeds;

    public ShotSetpoint(
        AngularVelocity flywheelSpeed,
        AngularVelocity feederSpeed,
        Angle hoodAngle,
        Pose2d robotPose,
        ChassisSpeeds robotSpeeds,
        Translation2d target) {
      this.flywheelSpeed = flywheelSpeed;
      this.feederSpeed = feederSpeed;
      this.hoodAngle = hoodAngle;
      this.robotPose = robotPose;
      this.robotSpeeds = robotSpeeds;
      this.target = target;
    }

    public ShotSetpoint() {
      this.flywheelSpeed = RotationsPerSecond.of(0);
      this.feederSpeed = RotationsPerSecond.of(0);
      this.hoodAngle = Degrees.of(0);
      this.robotPose = new Pose2d();
      this.robotSpeeds = new ChassisSpeeds();
      this.target = new Translation2d();
    }
  }
}
