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

  public static final double FLYWHEEL_ENERGY_TRANSFER_COEFFICENT = 60.0 / 40.0;

  public static AngularVelocity flywheelMPStoRPS(double mps) {
    double surfaceSpeed =
        mps * FLYWHEEL_ENERGY_TRANSFER_COEFFICENT; // compensates for energy from flywheel not fully
    // transfered
    double radius = Inches.of(2).in(Meters);
    double circumference = 2 * Math.PI * radius;
    double flywheelSpeed = surfaceSpeed / circumference;
    return RotationsPerSecond.of(flywheelSpeed);
  }

  public static ShotSetpoint createPolynomialShotSetpoint(
      Pose2d drivePose, ChassisSpeeds driveChassisSpeeds) {

    // Extract inputs from pose and speeds (robot-relative)
    double robotX = drivePose.getTranslation().getX();
    double robotY = drivePose.getTranslation().getY();
    double robotVx = driveChassisSpeeds.vxMetersPerSecond;
    double robotVy = driveChassisSpeeds.vyMetersPerSecond;

    // Build polynomial features (same as C implementation)
    double phi[] = new double[POLY_DIM];
    buildFeatures(robotX, robotY, robotVx, robotVy, phi);

    // Apply weights to get outputs
    double flywheelSpeedMps = dotProduct(W_SPEED, phi);
    double hoodAngleDeg = dotProduct(W_HOOD, phi);
    double chassisHeadingDeg = dotProduct(W_HEADING, phi);

    // Convert to WPILib units
    AngularVelocity flywheelRps = flywheelMPStoRPS(flywheelSpeedMps);
    Angle hoodAngle = Degrees.of(hoodAngleDeg);

    // Target is still the hub
    Translation2d hubTranslation2d = Constants.HUB_POSE_3D.getTranslation().toTranslation2d();

    // Robot pose should point at the computed chassis heading
    Pose2d robotPose =
        new Pose2d(drivePose.getTranslation(), Rotation2d.fromDegrees(chassisHeadingDeg));

    // Log the polynomial outputs
    Logger.recordOutput("ShotPlanner/polyFlywheelMps", flywheelSpeedMps);
    Logger.recordOutput("ShotPlanner/polyHoodDeg", hoodAngleDeg);
    Logger.recordOutput("ShotPlanner/polyChassisHeadingDeg", chassisHeadingDeg);

    return new ShotSetpoint(
        flywheelRps, hoodAngle, robotPose, driveChassisSpeeds, hubTranslation2d);
  }

  private static final int POLY_DIM = 15;

  // Thsese values are from a C program that runs regression on projectile simulation data
  // Fitted polynomial weights for flywheel speed
  public static double[] W_SPEED = {
    10.8554212665,
    -1.0794403611,
    -0.9860585121,
    -1.1082910120,
    -0.8064044977,
    0.0474293344,
    0.1409939368,
    0.0842794177,
    0.0995062485,
    0.0138711395,
    0.1314467387,
    0.0056830356,
    0.0086053290,
    0.2362094857,
    0.0029280495,
  };

  // Fitted polynomial weights for hood angle (deg)
  public static double[] W_HOOD = {
    40.2851914096,
    2.6479995152,
    4.5452348313,
    2.3932271699,
    3.4254217531,
    0.1008679675,
    -0.6225532038,
    -0.0904946126,
    -0.3465797840,
    -0.1099461808,
    0.1256604375,
    -0.0322133796,
    -0.0851354528,
    -0.9845063507,
    -0.0226411679,
  };

  // Fitted polynomial weights for chassis heading (deg)
  public static double[] W_HEADING = {
    -6.2120675691,
    22.0166611029,
    -1.0552058899,
    18.7660610805,
    -10.5761181435,
    -0.2133071513,
    0.4709395301,
    -0.1053182645,
    0.0966293136,
    -6.1080522176,
    -0.2967722323,
    -1.6689013859,
    -5.2924521412,
    0.4884681580,
    -1.8321625403,
  };

  // Build second-order polynomial features phi(x,y,vx,vy)
  private static void buildFeatures(double x, double y, double vx, double vy, double[] phi) {
    double x2 = x * x;
    double y2 = y * y;
    double vx2 = vx * vx;
    double vy2 = vy * vy;

    int k = 0;
    phi[k++] = 1.0; // bias
    phi[k++] = x;
    phi[k++] = y;
    phi[k++] = vx;
    phi[k++] = vy;

    phi[k++] = x2;
    phi[k++] = y2;
    phi[k++] = vx2;
    phi[k++] = vy2;

    phi[k++] = x * y;
    phi[k++] = x * vx;
    phi[k++] = x * vy;
    phi[k++] = y * vx;
    phi[k++] = y * vy;
    phi[k++] = vx * vy;
  }

  // Dot product phi · weights
  private static double dotProduct(double[] weights, double[] phi) {
    double result = 0.0;
    for (int i = 0; i < POLY_DIM; i++) {
      result += weights[i] * phi[i];
    }
    return result;
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
