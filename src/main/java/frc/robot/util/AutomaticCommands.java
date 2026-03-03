package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AutomaticCommands {
  public static Translation2d trenchBegin = new Translation2d(2.5, 8.25);
  public static Translation2d trenchEnd = new Translation2d(6.5, 6.25);
  public static Translation2d trenchNeutralEntry = new Translation2d(3.4, 7.58);
  public static Translation2d trenchAllianceEntry = new Translation2d(5.75, 7.58);

  public static Command trenchCommand(
      Drive drive, Translation2d a, Translation2d b, Pose2d currentPose) {
    Translation2d currentTrans = currentPose.getTranslation();
    Translation2d startPoint = currentTrans.getDistance(a) < currentTrans.getDistance(b) ? a : b;
    Translation2d endPoint = (startPoint == a) ? b : a;
    Logger.recordOutput("start", new Pose2d(a, Rotation2d.kZero));
    Logger.recordOutput("end", new Pose2d(b, Rotation2d.kZero));

    double currentAngle = currentPose.getRotation().getDegrees();
    double snappedAngle =
        Math.abs(Math.IEEEremainder(currentAngle, 360) - 180)
                < Math.abs(Math.IEEEremainder(currentAngle, 360))
            ? 0.0
            : 0.0;

    Pose2d startPose = new Pose2d(startPoint, Rotation2d.fromDegrees(snappedAngle));
    Pose2d endPose = new Pose2d(endPoint, Rotation2d.fromDegrees(snappedAngle));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, startPose, endPose);

    PathConstraints constraints = new PathConstraints(3, 3, 3, 3);

    var path =
        new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, drive.getRotation()));
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public static Command intakeUnderTowerCommand() {
    return Commands.none();
  }

  public static Command bumpCommand() {
    return Commands.none();
  }

  public static Command automaticCommand(Drive drive) {
    return Commands.defer(
        () -> {
          Pose2d pose = drive.getPose();

          // Check Trench Commands
          if (pose.getX() < 8 && pose.getY() > 4) {
            return trenchCommand(drive, trenchNeutralEntry, trenchAllianceEntry, pose);
          }
          if (pose.getX() < 8 && pose.getY() < 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipHorizontalMidline(trenchAllianceEntry),
                FieldFlipUtil.flipHorizontalMidline(trenchNeutralEntry),
                pose);
          }
          if (pose.getX() > 8 && pose.getY() > 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipVerticalMidline(trenchAllianceEntry),
                FieldFlipUtil.flipVerticalMidline(trenchNeutralEntry),
                pose);
          }
          if (pose.getX() > 8 && pose.getY() < 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipBothMidlines(trenchAllianceEntry),
                FieldFlipUtil.flipBothMidlines(trenchNeutralEntry),
                pose);
          }

          return Commands.none();
        },
        Set.of(drive));
  }

  public static boolean isPoseBetweenTranslations(
      Pose2d pose, Translation2d a, Translation2d b, double epsilonMeters) {
    return false;
  }

  public static boolean isPoseBetweenTranslations(Pose2d pose, Translation2d a, Translation2d b) {
    return isPoseBetweenTranslations(pose, a, b, 1e-3); // 1 mm tolerance
  }
}
