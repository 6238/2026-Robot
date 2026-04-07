package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AutomaticCommands {

  // PathPlanner constraints for all teleop pathfinding
  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(3.0, 3.5, Math.toRadians(540), Math.toRadians(720));

  // Trench
  public static final Translation2d trenchNeutralEntry = new Translation2d(5.75, 7.46);
  public static final Translation2d trenchAllianceEntry = new Translation2d(3.4, 7.46);

  // Hub back wall (TODO: measure on field)
  public static final Pose2d hubBackWallPose = new Pose2d(4.0, 5.5, Rotation2d.fromDegrees(180));

  // Wall scrape (TODO: measure on field)
  // Robot approaches at a slight angle so intake brushes the wall, then slides along it.
  public static final Pose2d wallScrapeStartPose = new Pose2d(1.5, 7.6, Rotation2d.fromDegrees(10));
  public static final Pose2d wallScrapeEndPose = new Pose2d(6.0, 7.6, Rotation2d.fromDegrees(10));

  // Y clearance from each side wall for the wall shoot setup
  public static final double WALL_SHOOT_SETUP_TOP_Y = 7.2;
  public static final double WALL_SHOOT_SETUP_BOTTOM_Y = 0.9;

  // Tower (TODO: measure on field)
  // Robot enters near the outpost side, then backs out.
  public static final Pose2d towerEntryPose = new Pose2d(4.0, 6.5, Rotation2d.fromDegrees(270));
  public static final Pose2d towerExitPose = new Pose2d(4.0, 4.5, Rotation2d.fromDegrees(90));

  /** Flips a Pose2d across the field's vertical midline (for red alliance). */
  private static Pose2d flipVertical(Pose2d pose) {
    return new Pose2d(FieldFlipUtil.flipVerticalMidline(pose.getTranslation()), pose.getRotation());
  }

  /**
   * Context-aware command for B button. If the robot is in the neutral zone (closer to the neutral
   * trench entry), drives to the position right under the trench. If the robot is in the alliance
   * zone (closer to the alliance trench entry), drives through the trench into the neutral zone.
   */
  public static Command automaticCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d pose = drive.getPose();
          // Bump zone: near the horizontal midline (y ~ 4.1) → go to hub back wall
          if (Math.abs(pose.getY() - 4.1) < 1.0) {
            return hubBackWallCommand(drive, driverOverride);
          }
          if (pose.getX() < 8 && pose.getY() > 4) {
            return trenchCommand(
                drive, trenchNeutralEntry, trenchAllianceEntry, pose, driverOverride);
          }
          if (pose.getX() < 8 && pose.getY() < 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipHorizontalMidline(trenchNeutralEntry),
                FieldFlipUtil.flipHorizontalMidline(trenchAllianceEntry),
                pose,
                driverOverride);
          }
          if (pose.getX() > 8 && pose.getY() > 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipVerticalMidline(trenchNeutralEntry),
                FieldFlipUtil.flipVerticalMidline(trenchAllianceEntry),
                pose,
                driverOverride);
          }
          return trenchCommand(
              drive,
              FieldFlipUtil.flipBothMidlines(trenchNeutralEntry),
              FieldFlipUtil.flipBothMidlines(trenchAllianceEntry),
              pose,
              driverOverride);
        },
        Set.of(drive));
  }

  /**
   * Context-aware trench command. If the robot is closer to the neutral entry it is in the neutral
   * zone and drives to the position right under the trench. If the robot is closer to the alliance
   * entry it is in the alliance zone and drives through the trench into the neutral zone.
   */
  public static Command trenchCommand(
      Drive drive,
      Translation2d neutralEntry,
      Translation2d allianceEntry,
      Pose2d currentPose,
      BooleanSupplier driverOverride) {
    Translation2d currentTrans = currentPose.getTranslation();
    boolean inNeutralZone =
        currentTrans.getDistance(allianceEntry) > currentTrans.getDistance(neutralEntry);

    Logger.recordOutput("trench/neutralEntry", new Pose2d(neutralEntry, Rotation2d.kZero));
    Logger.recordOutput("trench/allianceEntry", new Pose2d(allianceEntry, Rotation2d.kZero));
    Logger.recordOutput("trench/inNeutralZone", inNeutralZone);

    // Shooter faces toward the hub regardless of which side the robot starts on.
    Rotation2d shooterFacing =
        neutralEntry.getY() < 4 ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(270);
    // Shift Y 18 cm toward the field center to stay clear of the wall.
    double yOffset = allianceEntry.getY() > 4 ? -0.50 : 0.50;
    Translation2d adjustedAlliance =
        new Translation2d(allianceEntry.getX(), allianceEntry.getY() + yOffset);

    // Direction from alliance entry toward neutral entry (along the trench).
    Translation2d wallDir = neutralEntry.minus(allianceEntry);
    double wallDist = wallDir.getNorm();
    double wallVx = wallDir.getX() / wallDist * 0.5;
    double wallVy = wallDir.getY() / wallDist * 0.5;

    Translation2d adjustedNeutral =
        new Translation2d(neutralEntry.getX(), neutralEntry.getY() + yOffset);
    Pose2d alignPose = new Pose2d(neutralEntry, shooterFacing);
    Logger.recordOutput("AutomaticCommands/target", alignPose);

    if (inNeutralZone) {
      // Neutral zone: pathfind to neutral entry, drive into the wall, nudge off wall, then slide.
      return AutoBuilder.pathfindToPose(alignPose, CONSTRAINTS, 0.0)
          .until(driverOverride)
          .andThen(
              DriveCommands.joystickDriveRobotRelative(drive, () -> -0.5, () -> 0, () -> 0)
                  .withTimeout(0.3))
          // .andThen(drive.driveIntoWall(wallVx, wallVy, driverOverride))
          // .andThen(Commands.print("now!"))
          .andThen(
              DriveCommands.joystickDriveRobotRelative(drive, () -> 0.3, () -> 0, () -> 0)
                  .withTimeout(0.1))
          .andThen(
              DriveCommands.joystickDrive(drive, () -> -1, () -> 0, () -> 0).withTimeout(0.75));
    } else {
      // Alliance zone: robot points backwards (toward alliance wall) through the trench.
      Rotation2d backwardsFacing =
          neutralEntry.getX() < 8 ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
      Pose2d allianceAlignPose = new Pose2d(allianceEntry, backwardsFacing);
      Pose2d exitPose = new Pose2d(neutralEntry, backwardsFacing);
      Logger.recordOutput("AutomaticCommands/allianceEntry", allianceAlignPose);
      Logger.recordOutput("AutomaticCommands/exitPose", exitPose);
      return AutoBuilder.pathfindToPose(allianceAlignPose, CONSTRAINTS, 3.5)
          .until(driverOverride)
          .andThen(
              Commands.runOnce(() -> Logger.recordOutput("AutomaticCommands/target", exitPose)))
          .andThen(AutoBuilder.pathfindToPose(exitPose, CONSTRAINTS, 0.0).until(driverOverride));
    }
  }

  /** D-Pad Up: Lines up against the back wall of the hub. */
  public static Command hubBackWallCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d target =
              drive.getPose().getX() > 8 ? flipVertical(hubBackWallPose) : hubBackWallPose;
          Logger.recordOutput("AutomaticCommands/target", target);
          return AutoBuilder.pathfindToPose(target, CONSTRAINTS, 0.0).until(driverOverride);
        },
        Set.of(drive));
  }

  /**
   * D-Pad Down: Pathfinds to the nearest side wall at the robot's current x, ready to shoot while
   * drifting in the -x direction with D-Pad Right.
   */
  public static Command wallShootSetupCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d current = drive.getPose();
          double targetY =
              current.getY() > 4.1 ? WALL_SHOOT_SETUP_TOP_Y : WALL_SHOOT_SETUP_BOTTOM_Y;
          Pose2d setupPose = new Pose2d(current.getX(), targetY, current.getRotation());
          Logger.recordOutput("AutomaticCommands/target", setupPose);
          return AutoBuilder.pathfindToPose(setupPose, CONSTRAINTS, 0.0).until(driverOverride);
        },
        Set.of(drive));
  }

  /** D-Pad Left: Drives into the tower near the outpost, then back out. */
  public static Command underTowerCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          boolean isRedSide = drive.getPose().getX() > 8;
          Pose2d entryPose = isRedSide ? flipVertical(towerEntryPose) : towerEntryPose;
          Pose2d exitPose = isRedSide ? flipVertical(towerExitPose) : towerExitPose;
          Logger.recordOutput("AutomaticCommands/target", entryPose);
          return AutoBuilder.pathfindToPose(entryPose, CONSTRAINTS, 2.0)
              .until(driverOverride)
              .andThen(
                  Commands.runOnce(() -> Logger.recordOutput("AutomaticCommands/target", exitPose)))
              .andThen(
                  AutoBuilder.pathfindToPose(exitPose, CONSTRAINTS, 0.0).until(driverOverride));
        },
        Set.of(drive));
  }
}
