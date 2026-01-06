package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetection.ObjectDetection;
import frc.robot.subsystems.objectdetection.ObjectDetection.TrackedObject;
import java.util.ArrayList;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class AutoPilotUtils {
  public static final double PICKUP_STANDOFF_METERS = 0.71;

  private static ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveCommands.ANGLE_KP,
          0.0,
          DriveCommands.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));

  public static void initializeAutoPilot() {
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static record WallSegment(Pose2d start, Pose2d end) {}
  ;

  public static ArrayList<WallSegment> walls =
      new ArrayList<WallSegment>() {
        {
          add(
              new WallSegment(
                  new Pose2d(1.3, 0.0, new Rotation2d(0.0)),
                  new Pose2d(1.3, 13.0, new Rotation2d(0.0))));
        }
      };

  public static WallSegment findClosestWallSegment(Pose2d pose) {
    WallSegment closestSegment = null;
    double closestDistance = Double.MAX_VALUE;

    for (WallSegment segment : walls) {
      double distance = distanceToSegment(pose, segment);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestSegment = segment;
      }
    }

    return closestSegment;
  }

  private static double distanceToSegment(Pose2d pose, WallSegment segment) {
    double x1 = segment.start().getX();
    double y1 = segment.start().getY();
    double x2 = segment.end().getX();
    double y2 = segment.end().getY();
    double px = pose.getX();
    double py = pose.getY();

    double dx = x2 - x1;
    double dy = y2 - y1;
    double lengthSquared = dx * dx + dy * dy;

    if (lengthSquared == 0) {
      return pose.getTranslation().getDistance(segment.start().getTranslation());
    }

    double t = ((px - x1) * dx + (py - y1) * dy) / lengthSquared;
    t = Math.max(0, Math.min(1, t));

    double closestX = x1 + t * dx;
    double closestY = y1 + t * dy;

    return pose.getTranslation()
        .getDistance(new Pose2d(closestX, closestY, new Rotation2d()).getTranslation());
  }

  public static final double WALL_DISTANCE_PERPENDICULAR_THRESHOLD = 1.0;

  public static Rotation2d calculateEntryAngle(
      WallSegment closestSegment, Pose2d targetPose, Pose2d currentPose) {
    // if (distanceToSegment(targetPose, closestSegment) > WALL_DISTANCE_PERPENDICULAR_THRESHOLD) {
    // Wall is far enough away, approach directly to target. Return angle between
    // current pose and target pose
    return targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
    // }

    // Wall is to close, approach perpendicular to wall surface
    // this perpendicular direction depends on which side of the wall the robot is
    // on
    // Rotation2d wallDirection =
    //     closestSegment
    //         .end()
    //         .getTranslation()
    //         .minus(closestSegment.start().getTranslation())
    //         .getAngle();
    // return wallDirection.plus(
    //     new Rotation2d(
    //         Math.PI
    //             / 2.0
    //             * Math.signum(
    //                 (currentPose.getX() - closestSegment.start().getX()) * wallDirection.getCos()
    //                     - (currentPose.getY() - closestSegment.start().getY())
    //                         * wallDirection.getSin())));
  }

  public static Pose2d computeStandoffPose(Drive drivetrain, TrackedObject targetObject) {
    Pose2d targetPose = targetObject.getPose();

    WallSegment closestWallSegment = findClosestWallSegment(targetPose);
    Rotation2d entryAngle =
        calculateEntryAngle(closestWallSegment, targetPose, drivetrain.getPose());

    Logger.recordOutput("AutoPilotPathing/entryAngle", entryAngle);
    Logger.recordOutput("AutoPilotPathing/targetPose", targetPose);

    double backX = -PICKUP_STANDOFF_METERS * entryAngle.getCos();
    double backY = -PICKUP_STANDOFF_METERS * entryAngle.getSin();

    Pose2d standoffPose =
        new Pose2d(targetPose.getX() + backX, targetPose.getY() + backY, entryAngle);

    Logger.recordOutput("AutoPilotPathing/standoffPose", standoffPose);

    return standoffPose;
  }

  private static Optional<TrackedObject> getCurrentTarget(
      Drive drivetrain, ObjectDetection objectDetection) {
    return objectDetection.closestTrackedObject(drivetrain.getPose());
  }

  public static Command generateNotePickupMoveCommand(Drive drivetrain, Pose2d standoffPose) {

    Rotation2d entryAngle = standoffPose.getRotation();
    APTarget target = new APTarget(standoffPose).withEntryAngle(entryAngle);

    return drivetrain
        .run(
            () -> {
              ChassisSpeeds robotRelativeSpeeds = drivetrain.getChassisSpeeds();
              Pose2d pose = drivetrain.getPose();

              APResult output = Drive.kAutopilot.calculate(pose, robotRelativeSpeeds, target);

              LinearVelocity veloX = output.vx();
              LinearVelocity veloY = output.vy();
              Rotation2d headingReference = entryAngle;

              double omega =
                  angleController.calculate(
                      drivetrain.getRotation().getRadians(), headingReference.getRadians());

              ChassisSpeeds speeds =
                  new ChassisSpeeds(veloX.in(MetersPerSecond), veloY.in(MetersPerSecond), omega);

              drivetrain.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getRotation()));
            })
        .until(() -> Drive.kAutopilot.atTarget(drivetrain.getPose(), target))
        .beforeStarting(() -> angleController.reset(drivetrain.getRotation().getRadians()));
  }

  public static Command generateIterativePickupCommand(
      Drive drivetrain, ObjectDetection objectDetection, CommandXboxController controller) {

    final double REPLAN_DISTANCE = 0.1;
    final double SUCCESS_DISTANCE = 0.06;
    final double MAX_TARGET_LOST_TIME = 0.5;

    // Mutable state local to the command
    AtomicReference<Pose2d> previousStandoffPose = new AtomicReference<>();
    AtomicReference<Double> lastSeenTargetTime = new AtomicReference<>(Timer.getFPGATimestamp());
    AtomicReference<Boolean> targetReached = new AtomicReference<>(false);

    Command oneSegment =
        Commands.defer(
            () -> {
              Optional<TrackedObject> optTarget = getCurrentTarget(drivetrain, objectDetection);
              if (optTarget.isEmpty()) {
                return Commands.none();
              }

              TrackedObject trackedObject = optTarget.get();
              Pose2d standoffPose = computeStandoffPose(drivetrain, trackedObject);

              previousStandoffPose.set(standoffPose);
              lastSeenTargetTime.set(Timer.getFPGATimestamp());

              return generateNotePickupMoveCommand(drivetrain, standoffPose)
                  .until(
                      () -> {
                        Logger.recordOutput("lastSeenTargetTime", lastSeenTargetTime.get());

                        Optional<TrackedObject> currentOpt =
                            getCurrentTarget(drivetrain, objectDetection);

                        double now = Timer.getFPGATimestamp();

                        if (currentOpt.isEmpty()) {
                          return (now - lastSeenTargetTime.get()) > MAX_TARGET_LOST_TIME;
                        }

                        TrackedObject currentTracked = currentOpt.get();
                        Pose2d currentStandoffPose =
                            computeStandoffPose(drivetrain, currentTracked);
                        Pose2d prevStandoff = previousStandoffPose.get();

                        Logger.recordOutput(
                            "AutoPilotPathing/currentStandoff", currentStandoffPose);
                        Logger.recordOutput("AutoPilotPathing/prevStandoff", prevStandoff);

                        lastSeenTargetTime.set(now);

                        Logger.recordOutput(
                            "AutoPilotPathing/distance",
                            currentStandoffPose
                                .getTranslation()
                                .getDistance(drivetrain.getPose().getTranslation()));
                        boolean atTarget =
                            currentStandoffPose
                                    .getTranslation()
                                    .getDistance(drivetrain.getPose().getTranslation())
                                < SUCCESS_DISTANCE;

                        if (atTarget) {
                          targetReached.set(true);
                          return true;
                        } else {
                          targetReached.set(false);
                        }

                        boolean needReplan =
                            currentStandoffPose
                                    .getTranslation()
                                    .getDistance(prevStandoff.getTranslation())
                                > REPLAN_DISTANCE;

                        if (needReplan) {
                          previousStandoffPose.set(currentStandoffPose);
                          DataLogManager.log("new standoff pose (replan)");
                          Logger.recordOutput("AutoPilotPathing/newStandoff", currentStandoffPose);
                        }

                        return needReplan;
                      });
            },
            Set.of(drivetrain));

    // Add .until() AFTER .repeatedly() to actually stop the loop
    return oneSegment
        .repeatedly()
        .until(
            () -> {
              // Stop if target reached
              if (targetReached.get()) {
                return true;
              }

              // Stop if target lost for too long
              double now = Timer.getFPGATimestamp();
              boolean targetLost = (now - lastSeenTargetTime.get()) > MAX_TARGET_LOST_TIME;

              if (targetLost) {
                CommandScheduler.getInstance()
                    .schedule(
                        Commands.sequence(
                            Commands.sequence(
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kLeftRumble, 1)),
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kLeftRumble, 0)),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kRightRumble, 1)),
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kRightRumble, 0))),
                            Commands.sequence(
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kLeftRumble, 1)),
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kLeftRumble, 0)),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kRightRumble, 1)),
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kRightRumble, 0))),
                            Commands.sequence(
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kLeftRumble, 1)),
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kLeftRumble, 0)),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kRightRumble, 1)),
                                Commands.waitSeconds(0.1),
                                Commands.runOnce(
                                    () -> controller.setRumble(RumbleType.kRightRumble, 0)))));
              }

              return targetLost;
            });
  }
}
