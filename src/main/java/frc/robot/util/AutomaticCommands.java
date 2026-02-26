package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
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

public class AutomaticCommands {
  public static Translation2d trenchBegin = new Translation2d(2.5, 8.25);
  public static Translation2d trenchEnd = new Translation2d(6.5, 6.25);
  public static Translation2d trenchNeutralEntry = new Translation2d(3.4, 7.5);
  public static Translation2d trenchAllianceEntry = new Translation2d(5.75, 7.5);

  public static Command trenchCommand(
      Drive drive, Translation2d a, Translation2d b, Pose2d currentPose) {
    Translation2d currentTrans = currentPose.getTranslation();
    Translation2d startPoint = currentTrans.getDistance(a) < currentTrans.getDistance(b) ? a : b;
    Translation2d endPoint = (startPoint == a) ? b : a;

    double currentAngle = currentPose.getRotation().getDegrees();
    double snappedAngle =
        Math.abs(Math.IEEEremainder(currentAngle, 360) - 180)
                < Math.abs(Math.IEEEremainder(currentAngle, 360))
            ? 180.0
            : 0.0;

    Pose2d startPose = new Pose2d(startPoint, Rotation2d.fromDegrees(snappedAngle));
    Pose2d endPose = new Pose2d(endPoint, Rotation2d.fromDegrees(snappedAngle));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, startPose, endPose);

    PathConstraints constraints = new PathConstraints(1, 1, 1, 1);

    return AutoBuilder.followPath(
        new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(0, drive.getRotation()),
            new GoalEndState(0, Rotation2d.fromDegrees(snappedAngle))));
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
          if (isPoseBetweenTranslations(pose, trenchBegin, trenchEnd)) {
            return trenchCommand(drive, trenchNeutralEntry, trenchAllianceEntry, pose);
          }
          if (isPoseBetweenTranslations(
              pose,
              FieldFlipUtil.flipHorizontalMidline(trenchBegin),
              FieldFlipUtil.flipHorizontalMidline(trenchEnd))) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipHorizontalMidline(trenchBegin),
                FieldFlipUtil.flipHorizontalMidline(trenchEnd),
                pose);
          }
          if (isPoseBetweenTranslations(
              pose,
              FieldFlipUtil.flipVerticalMidline(trenchBegin),
              FieldFlipUtil.flipVerticalMidline(trenchEnd))) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipVerticalMidline(trenchBegin),
                FieldFlipUtil.flipVerticalMidline(trenchEnd),
                pose);
          }
          if (isPoseBetweenTranslations(
              pose,
              FieldFlipUtil.flipBothMidlines(trenchBegin),
              FieldFlipUtil.flipBothMidlines(trenchEnd))) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipBothMidlines(trenchBegin),
                FieldFlipUtil.flipBothMidlines(trenchEnd),
                pose);
          }

          return Commands.none();
        },
        Set.of(drive));
  }

  public static boolean isPoseBetweenTranslations(
      Pose2d pose, Translation2d a, Translation2d b, double epsilonMeters) {

    Translation2d p = pose.getTranslation();

    double abx = b.getX() - a.getX();
    double aby = b.getY() - a.getY();
    double apx = p.getX() - a.getX();
    double apy = p.getY() - a.getY();

    double abLen2 = abx * abx + aby * aby;
    if (abLen2 < 1e-9) {
      double dx = p.getX() - a.getX();
      double dy = p.getY() - a.getY();
      return dx * dx + dy * dy <= epsilonMeters * epsilonMeters;
    }

    double dot = apx * abx + apy * aby;
    double t = dot / abLen2;

    // if projection falls outside the segment, it's not between
    if (t < 0.0 || t > 1.0) {
      return false;
    }

    double closestX = a.getX() + t * abx;
    double closestY = a.getY() + t * aby;

    double dx = p.getX() - closestX;
    double dy = p.getY() - closestY;
    double dist2 = dx * dx + dy * dy;

    return dist2 <= epsilonMeters * epsilonMeters;
  }

  public static boolean isPoseBetweenTranslations(Pose2d pose, Translation2d a, Translation2d b) {
    return isPoseBetweenTranslations(pose, a, b, 1e-3); // 1 mm tolerance
  }
}
