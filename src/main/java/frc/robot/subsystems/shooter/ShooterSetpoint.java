package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;

public class ShooterSetpoint {
    public AngularVelocity flywheelSpeed;
    public Angle hoodAngle;
    public Pose2d robotPose;
    public ChassisSpeeds robotSpeeds;

    public ShooterSetpoint(
            AngularVelocity flywheelSpeed,
            Angle hoodAngle,
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds) {
        this.flywheelSpeed = flywheelSpeed;
        this.hoodAngle = hoodAngle;
        this.robotPose = robotPose;
        this.robotSpeeds = robotSpeeds;
    }

    // public static ShooterSetpoint generateNearestShooterSetpoint(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Time leadTimeSec) {
    //     AngularVelocity flywheelSpeed = ShooterConstants.SHOOTER_LOOKUP_TABLE.get(
    //             Meters.of(robotPose.getTranslation().getDistance(Constants.HUB_POSE_3D.getTranslation().toTranslation2d())));
        
    //     Pose2d lookAtHubPose2d = 
    //             new Pose2d(
    //                     robotPose.getTranslation(),
    //                     Constants.HUB_POSE_3D.getTranslation().toTranslation2d().minus(robotPose.getTranslation()).getAngle());

    //     return new ShooterSetpoint(
    //             flywheelSpeed, ShooterConstants.FIXED_HOOD_ANGLE_DEGREES, lookAtHubPose2d, new ChassisSpeeds());
    // }

    
    // public static ShooterSetpoint generateNearestShooterSetpoint(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds, Time leadTime) {
    //     ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
    //         robotRelativeSpeeds.vxMetersPerSecond,
    //         robotRelativeSpeeds.vyMetersPerSecond,
    //         robotRelativeSpeeds.omegaRadiansPerSecond,
    //         robotPose.getRotation()
    //     );

    //     Translation2d predictedTranslation = robotPose.getTranslation().plus(
    //         new Translation2d(
    //             fieldSpeeds.vxMetersPerSecond * leadTime.in(Seconds),
    //             fieldSpeeds.vyMetersPerSecond * leadTime.in(Seconds)
    //         )
    //     );

    //     Pose2d predicPose2d = new Pose2d(predictedTranslation, robotPose.getRotation());

    //     Translation2d hubTranslation2d = Constants.HUB_POSE_3D.getTranslation().toTranslation2d();
    //     double predictedDistanceM = predicPose2d.getTranslation().getDistance(hubTranslation2d);
                
    //     AngularVelocity flywheelSpeed = ShooterConstants.SHOOTER_LOOKUP_TABLE.get(Meters.of(predictedDistanceM));
        
    //     Pose2d lookAtHubPose2d = 
    //         new Pose2d(
    //             robotPose.getTranslation(),
    //             hubTranslation2d.minus(predictedTranslation).getAngle()
    //         );

    //     return new ShooterSetpoint(
    //         flywheelSpeed, ShooterConstants.FIXED_HOOD_ANGLE_DEGREES, lookAtHubPose2d, robotRelativeSpeeds);
    // }

    public static ShooterSetpoint generateNearestShooterSetpoint(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            robotPose.getRotation()
        );

        Translation2d hubTranslation2d = Constants.HUB_POSE_3D.getTranslation().toTranslation2d();
        double distanceCurrently = robotPose.getTranslation().getDistance(hubTranslation2d);
        double leadTimeSec = ShooterConstants.SHOOTER_LEAD_TIME_FROM_DIST.get(distanceCurrently);

        Translation2d predictedTranslation = robotPose.getTranslation().plus(
            new Translation2d(
                fieldSpeeds.vxMetersPerSecond * leadTimeSec,
                fieldSpeeds.vyMetersPerSecond * leadTimeSec
            )
        );

        Pose2d predicPose2d = new Pose2d(predictedTranslation, robotPose.getRotation());

        double predictedDistanceM = predicPose2d.getTranslation().getDistance(hubTranslation2d);
                
        AngularVelocity flywheelSpeed = ShooterConstants.SHOOTER_LOOKUP_TABLE.get(Meters.of(predictedDistanceM));
        
        Pose2d lookAtHubPose2d = 
            new Pose2d(
                robotPose.getTranslation(),
                hubTranslation2d.minus(predictedTranslation).getAngle()
            );

        return new ShooterSetpoint(
            flywheelSpeed, ShooterConstants.FIXED_HOOD_ANGLE_DEGREES, lookAtHubPose2d, robotRelativeSpeeds);
    }
}
