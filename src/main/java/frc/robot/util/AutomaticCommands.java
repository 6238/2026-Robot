package frc.robot.util;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class AutomaticCommands {

  // Trench
  public static final Translation2d trenchNeutralEntry = new Translation2d(3.4, 7.58);
  public static final Translation2d trenchAllianceEntry = new Translation2d(5.75, 7.58);

  // Bump (TODO: measure on field)
  public static final Pose2d bumpPose = new Pose2d(4.0, 4.1, Rotation2d.fromDegrees(0));

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
   * Context-aware command for B button. Navigates through the trench or over the bump depending on
   * the robot's current field position.
   */
  public static Command automaticCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d pose = drive.getPose();
          // Bump detection: near the horizontal midline (y ~ 4.1)
          if (Math.abs(pose.getY() - 4.1) < 1.0) {
            return bumpCommand(drive, driverOverride);
          }
          // Trench navigation based on quadrant
          if (pose.getX() < 8 && pose.getY() > 4) {
            return trenchCommand(
                drive, trenchNeutralEntry, trenchAllianceEntry, pose, driverOverride);
          }
          if (pose.getX() < 8 && pose.getY() < 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipHorizontalMidline(trenchAllianceEntry),
                FieldFlipUtil.flipHorizontalMidline(trenchNeutralEntry),
                pose,
                driverOverride);
          }
          if (pose.getX() > 8 && pose.getY() > 4) {
            return trenchCommand(
                drive,
                FieldFlipUtil.flipVerticalMidline(trenchAllianceEntry),
                FieldFlipUtil.flipVerticalMidline(trenchNeutralEntry),
                pose,
                driverOverride);
          }
          return trenchCommand(
              drive,
              FieldFlipUtil.flipBothMidlines(trenchAllianceEntry),
              FieldFlipUtil.flipBothMidlines(trenchNeutralEntry),
              pose,
              driverOverride);
        },
        Set.of(drive));
  }

  /** Navigates through the trench from the nearest entry point to the far end. */
  public static Command trenchCommand(
      Drive drive,
      Translation2d a,
      Translation2d b,
      Pose2d currentPose,
      BooleanSupplier driverOverride) {
    Translation2d currentTrans = currentPose.getTranslation();
    Translation2d startPoint = currentTrans.getDistance(a) < currentTrans.getDistance(b) ? a : b;
    Translation2d endPoint = (startPoint == a) ? b : a;

    APTarget entryTarget =
        new APTarget(new Pose2d(startPoint, currentPose.getRotation())).withVelocity(1.0);
    APTarget exitTarget =
        new APTarget(new Pose2d(endPoint, currentPose.getRotation())).withVelocity(0.0);

    return drive
        .align(entryTarget, driverOverride)
        .andThen(drive.align(exitTarget, driverOverride));
  }

  /** Navigates the robot over the bump to the target crossing pose. */
  public static Command bumpCommand(Drive drive, BooleanSupplier driverOverride) {
    return drive.align(new APTarget(bumpPose).withVelocity(0.0), driverOverride);
  }

  /** D-Pad Up: Lines up against the back wall of the hub. */
  public static Command hubBackWallCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d target =
              drive.getPose().getX() > 8 ? flipVertical(hubBackWallPose) : hubBackWallPose;
          return drive.align(new APTarget(target).withVelocity(0.0), driverOverride);
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
          return drive.align(new APTarget(setupPose).withVelocity(0.0), driverOverride);
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
          return drive
              .align(new APTarget(entryPose).withVelocity(0.0), driverOverride)
              .andThen(drive.align(new APTarget(exitPose).withVelocity(0.0), driverOverride));
        },
        Set.of(drive));
  }
}
