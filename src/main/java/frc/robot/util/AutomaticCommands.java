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

  private static final PathConstraints CONSTRAINTS =
      new PathConstraints(3.0, 3.5, Math.toRadians(540), Math.toRadians(720));

  // Slower constraints for final approaches into the trench
  private static final PathConstraints TRENCH_APPROACH_CONSTRAINTS =
      new PathConstraints(1.25, 3, Math.toRadians(540), Math.toRadians(720));

  // Trench geometry
  public static final Translation2d trenchNeutralEntry = new Translation2d(5.7, 7.46);
  public static final Translation2d trenchAllianceEntry = new Translation2d(3.3, 7.46);

  // One bump on the blue-alliance, top-half side. Two crossing poses: alliance side and neutral
  // side.
  // Mirrored over each midline → 4 bumps total on the field.
  private static final Pose2d bumpAllianceSide =
      new Pose2d(3.546, 5.455, Rotation2d.fromDegrees(0));
  private static final Pose2d bumpNeutralSide = new Pose2d(5.654, 5.455, Rotation2d.fromDegrees(0));
  private static final double BUMP_DETECTION_RADIUS = 3.5;

  // Trench wall lineup only activates within this Y distance of the outer wall (trench is at 7.46)
  private static final double TRENCH_Y_ACTIVATION = 6.0;

  // Hub back wall (TODO: measure on field)
  public static final Pose2d hubBackWallPose = new Pose2d(4.0, 5.5, Rotation2d.fromDegrees(180));

  // Wall scrape (TODO: measure on field)
  public static final Pose2d wallScrapeStartPose = new Pose2d(1.5, 7.6, Rotation2d.fromDegrees(10));
  public static final Pose2d wallScrapeEndPose = new Pose2d(6.0, 7.6, Rotation2d.fromDegrees(10));

  public static final double WALL_SHOOT_SETUP_TOP_Y = 7.2;
  public static final double WALL_SHOOT_SETUP_BOTTOM_Y = 0.9;

  // Wall tower traversal (B button, near alliance wall)
  public static final Translation2d wallTowerTopEntry = new Translation2d(0.528, 5.117);
  public static final Translation2d wallTowerBottomEntry = new Translation2d(0.528, 2.598);

  // Under-tower (D-Pad Left, TODO: measure on field)
  public static final Pose2d towerEntryPose = new Pose2d(4.0, 6.5, Rotation2d.fromDegrees(270));
  public static final Pose2d towerExitPose = new Pose2d(4.0, 4.5, Rotation2d.fromDegrees(90));

  private static Pose2d flipVertical(Pose2d pose) {
    return new Pose2d(FieldFlipUtil.flipVerticalMidline(pose.getTranslation()), pose.getRotation());
  }

  // ── B Button ─────────────────────────────────────────────────────────────

  /**
   * B button: context-aware command.
   *
   * <p>Near the alliance wall (x &lt; 2.0 or x &gt; 14.0): traverses the wall tower.
   *
   * <p>Otherwise: trench traversal — snaps heading to nearest 0° or 180° and pathfinds between
   * neutral and alliance entries.
   *
   * <p>Special case: if the robot is already inside the trench corridor, back out first.
   */
  public static Command automaticCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d pose = drive.getPose();
          // Wall tower (near own alliance wall)
          if (pose.getX() < 2.0) {
            return wallTowerCommand(
                drive, wallTowerTopEntry, wallTowerBottomEntry, pose, driverOverride);
          }
          if (pose.getX() > 14.0) {
            return wallTowerCommand(
                drive,
                FieldFlipUtil.flipVerticalMidline(wallTowerTopEntry),
                FieldFlipUtil.flipVerticalMidline(wallTowerBottomEntry),
                pose,
                driverOverride);
          }
          // Trench
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
   * B button wall tower traversal: pathfinds to the nearer of the two tower waypoints, then
   * continues through to the far waypoint.
   */
  private static Command wallTowerCommand(
      Drive drive,
      Translation2d topEntry,
      Translation2d bottomEntry,
      Pose2d currentPose,
      BooleanSupplier driverOverride) {

    boolean closerToTop =
        currentPose.getTranslation().getDistance(topEntry)
            < currentPose.getTranslation().getDistance(bottomEntry);
    Translation2d fromEntry = closerToTop ? topEntry : bottomEntry;
    Translation2d toEntry = closerToTop ? bottomEntry : topEntry;

    // Intake faces direction of motion: moving toward higher y → intake faces +y → heading 270°
    Rotation2d traversalHeading =
        toEntry.getY() >= fromEntry.getY()
            ? Rotation2d.fromDegrees(270)
            : Rotation2d.fromDegrees(90);

    Pose2d fromPose = new Pose2d(fromEntry, traversalHeading);
    Logger.recordOutput("AutomaticCommands/wallTower/fromPose", fromPose);
    Logger.recordOutput(
        "AutomaticCommands/wallTower/toPose", new Pose2d(toEntry, traversalHeading));

    // Drive field-relative along Y; time = distance / (0.5 joystick * ~3 m/s max)
    double yDir = toEntry.getY() > fromEntry.getY() ? 1.0 : -1.0;
    double traversalSeconds = fromEntry.getDistance(toEntry) / 1.5;

    return AutoBuilder.pathfindToPose(fromPose, TRENCH_APPROACH_CONSTRAINTS, 0.0)
        .until(driverOverride)
        .andThen(
            DriveCommands.joystickDrive(drive, () -> 0.0, () -> yDir * 0.5, () -> 0.0)
                .withTimeout(traversalSeconds)
                .until(driverOverride));
  }

  /**
   * B button trench traversal: snaps the robot to the nearest 0° or 180° heading, pathfinds to the
   * entry point, then drives through to the exit point.
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
    boolean inCorridor = inTrenchCorridor(currentPose, neutralEntry, allianceEntry);

    Logger.recordOutput("trench/neutralEntry", new Pose2d(neutralEntry, Rotation2d.kZero));
    Logger.recordOutput("trench/allianceEntry", new Pose2d(allianceEntry, Rotation2d.kZero));
    Logger.recordOutput("trench/inNeutralZone", inNeutralZone);
    Logger.recordOutput("trench/inCorridor", inCorridor);

    Translation2d fromEntry = inNeutralZone ? neutralEntry : allianceEntry;
    Translation2d toEntry = inNeutralZone ? allianceEntry : neutralEntry;

    // Snap current heading to nearest 0° or 180°
    double headingDeg = ((currentPose.getRotation().getDegrees() % 360) + 360) % 360;
    Rotation2d traversalHeading =
        (headingDeg < 90 || headingDeg >= 270)
            ? Rotation2d.fromDegrees(0)
            : Rotation2d.fromDegrees(180);

    Pose2d fromPose = new Pose2d(fromEntry, traversalHeading);
    Pose2d toPose = new Pose2d(toEntry, traversalHeading);
    Logger.recordOutput("AutomaticCommands/trench/fromPose", fromPose);
    Logger.recordOutput("AutomaticCommands/trench/toPose", toPose);

    Command mainSeq =
        AutoBuilder.pathfindToPose(fromPose, TRENCH_APPROACH_CONSTRAINTS, 1.25)
            .until(driverOverride)
            .andThen(AutoBuilder.pathfindToPose(toPose, CONSTRAINTS, 1.0).until(driverOverride));

    if (inCorridor) {
      return AutoBuilder.pathfindToPose(
              trenchClearPose(currentPose, neutralEntry, allianceEntry), CONSTRAINTS, 0.0)
          .until(driverOverride)
          .andThen(mainSeq);
    }
    return mainSeq;
  }

  // ── X Button ─────────────────────────────────────────────────────────────

  /**
   * X button: context-aware.
   *
   * <p>Near the horizontal midline (|y - 4.1| &lt; 1.0): navigates over the bump.
   *
   * <p>Otherwise: trench wall lineup — pathfinds to neutral entry with intake facing the outer
   * wall, then backs into the wall to localize.
   */
  public static Command neutralToAllianceCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d pose = drive.getPose();
          double fieldHeight = 8.21;
          boolean nearTopWall = pose.getY() > TRENCH_Y_ACTIVATION;
          boolean nearBottomWall = pose.getY() < fieldHeight - TRENCH_Y_ACTIVATION;
          if (pose.getX() < 8 && nearTopWall) {
            return trenchWallLineup(drive, trenchNeutralEntry, driverOverride);
          }
          if (pose.getX() < 8 && nearBottomWall) {
            return trenchWallLineup(
                drive, FieldFlipUtil.flipHorizontalMidline(trenchNeutralEntry), driverOverride);
          }
          if (pose.getX() > 8 && nearTopWall) {
            return trenchWallLineup(
                drive, FieldFlipUtil.flipVerticalMidline(trenchNeutralEntry), driverOverride);
          }
          if (pose.getX() > 8 && nearBottomWall) {
            return trenchWallLineup(
                drive, FieldFlipUtil.flipBothMidlines(trenchNeutralEntry), driverOverride);
          }
          if (nearBump(pose)) {
            return bumpCommand(drive, driverOverride);
          }
          return Commands.none();
        },
        Set.of(drive));
  }

  /** Navigates the robot over the bump to the target crossing pose. */
  public static Command bumpCommand(Drive drive, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Pose2d target = nearestBump(drive.getPose());
          Logger.recordOutput("AutomaticCommands/target", target);
          return AutoBuilder.pathfindToPose(target, CONSTRAINTS, 0.0).until(driverOverride);
        },
        Set.of(drive));
  }

  /**
   * Pathfinds to the neutral entry at ±90° (shooter toward field center, intake toward outer wall),
   * then backs the intake into the wall for localization.
   */
  private static Command trenchWallLineup(
      Drive drive, Translation2d neutralEntry, BooleanSupplier driverOverride) {
    // Shooter faces field center so intake faces the outer wall
    Rotation2d shooterFacing =
        neutralEntry.getY() < 4 ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(270);
    // Small offset toward field center so the wall press reliably reaches the wall
    double yOffset = neutralEntry.getY() > 4 ? -0.3 : 0.3;
    Pose2d alignPose =
        new Pose2d(neutralEntry.getX(), neutralEntry.getY() + yOffset, shooterFacing);
    Logger.recordOutput("AutomaticCommands/target", alignPose);

    // Robot is at 90°/270° after lineup, so robot-relative Y = field X axis.
    // Compute the robot-Y direction that points toward the alliance zone.
    double fieldXToAlliance = neutralEntry.getX() < 8 ? -1.0 : 1.0;
    double robotYDir = fieldXToAlliance * (-shooterFacing.getSin());
    double driveSpeed = 1.4 / 3.0; // 1.4 m/s as fraction of ~3 m/s max

    return AutoBuilder.pathfindToPose(alignPose, TRENCH_APPROACH_CONSTRAINTS, 0.0)
        .until(driverOverride)
        .andThen(
            DriveCommands.joystickDriveRobotRelative(drive, () -> -0.5, () -> 0, () -> 0)
                .withTimeout(0.3))
        .andThen(
            DriveCommands.joystickDriveRobotRelative(drive, () -> 0.3, () -> 0, () -> 0)
                .withTimeout(0.1))
        .andThen(
            DriveCommands.joystickDriveRobotRelative(
                    drive, () -> 0.0, () -> robotYDir * driveSpeed, () -> 0.0)
                .withTimeout(0.5));
  }

  // ── Other commands ────────────────────────────────────────────────────────

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

  // ── Helpers ───────────────────────────────────────────────────────────────

  /**
   * Returns the bump crossing pose the robot should target. The field has 4 bumps (one pose
   * mirrored over each midline). Each bump has an alliance-side and neutral-side crossing point;
   * the robot navigates to whichever side it is currently on.
   */
  private static Pose2d nearestBump(Pose2d robotPose) {
    Pose2d allianceSide = bumpAllianceSide;
    Pose2d neutralSide = bumpNeutralSide;
    boolean redSide = robotPose.getX() > 8;
    if (robotPose.getY() < 4.105) {
      allianceSide =
          new Pose2d(
              FieldFlipUtil.flipHorizontalMidline(allianceSide.getTranslation()),
              allianceSide.getRotation());
      neutralSide =
          new Pose2d(
              FieldFlipUtil.flipHorizontalMidline(neutralSide.getTranslation()),
              neutralSide.getRotation());
    }
    if (redSide) {
      allianceSide =
          new Pose2d(
              FieldFlipUtil.flipVerticalMidline(allianceSide.getTranslation()),
              allianceSide.getRotation());
      neutralSide =
          new Pose2d(
              FieldFlipUtil.flipVerticalMidline(neutralSide.getTranslation()),
              neutralSide.getRotation());
    }
    // Navigate to the side of the bump that the robot is currently on
    double bumpMidX = (allianceSide.getX() + neutralSide.getX()) / 2.0;
    boolean inAllianceZone = redSide ? robotPose.getX() > bumpMidX : robotPose.getX() < bumpMidX;
    return inAllianceZone ? neutralSide : allianceSide;
  }

  /** Returns true if the robot is within BUMP_DETECTION_RADIUS of the nearest bump. */
  private static boolean nearBump(Pose2d robotPose) {
    return robotPose.getTranslation().getDistance(nearestBump(robotPose).getTranslation())
        < BUMP_DETECTION_RADIUS;
  }

  /**
   * Returns true if the robot is inside the trench corridor and close enough to the wall that it
   * should back out before running a trench command.
   */
  private static boolean inTrenchCorridor(
      Pose2d currentPose, Translation2d neutralEntry, Translation2d allianceEntry) {
    double minX = Math.min(neutralEntry.getX(), allianceEntry.getX()) - 0.3;
    double maxX = Math.max(neutralEntry.getX(), allianceEntry.getX()) + 0.3;
    double trenchY = neutralEntry.getY();
    return currentPose.getX() > minX
        && currentPose.getX() < maxX
        && Math.abs(currentPose.getY() - trenchY) < 0.4;
  }

  /**
   * Returns a pose clear of the trench corridor: 0.7 m toward field center in Y, and exits in X
   * toward whichever end of the trench the robot is closest to.
   */
  private static Pose2d trenchClearPose(
      Pose2d currentPose, Translation2d neutralEntry, Translation2d allianceEntry) {
    double trenchY = neutralEntry.getY();
    double safeY = trenchY + (trenchY > 4.1 ? -0.7 : 0.7);
    double minX = Math.min(neutralEntry.getX(), allianceEntry.getX());
    double maxX = Math.max(neutralEntry.getX(), allianceEntry.getX());
    double midX = (minX + maxX) / 2.0;
    double safeX = currentPose.getX() < midX ? minX - 0.7 : maxX + 0.7;
    return new Pose2d(safeX, safeY, currentPose.getRotation());
  }
}
