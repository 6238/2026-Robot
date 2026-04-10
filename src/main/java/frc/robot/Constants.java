// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final int MAX_PHEONIX_RETRIES = 5;

  public final class PathfindingConfig {
    public static final double DRIVE_RESUME_DEADBAND = 0.2;
  }

  public static final DoubleFunction<Boolean> SHOULD_PASS =
      (poseX) -> {
        if (RobotContainer.isRed()) {
          return poseX < 11.5;
        }
        return poseX > 4.75;
      };

  public static final Supplier<Pose3d> HUB_POSE_3D =
      () -> {
        if (RobotContainer.isRed()) {
          return new Pose3d(11.932, 4.147, 0.0, Rotation3d.kZero);
        }
        return new Pose3d(4.625, 4.045, 0.0, Rotation3d.kZero);
      };

  public static final Supplier<Pose2d> LEFT_TARGET_PASS_POSE2D =
      () -> {
        if (RobotContainer.isRed()) {
          return new Pose2d(14, 5.75, Rotation2d.kZero);
        }
        return new Pose2d(1.75, 5.75, Rotation2d.kZero);
      };

  public static final Supplier<Pose2d> RIGHT_TARGET_PASS_POSE2D =
      () -> {
        if (RobotContainer.isRed()) {
          return new Pose2d(14, 2, Rotation2d.kZero);
        }
        return new Pose2d(1.75, 2, Rotation2d.kZero);
      };
  public static final double LEFT_RIGHT_SPLIT = Constants.HUB_POSE_3D.get().getY();

  /**
   * When true, suppresses non-essential Logger.recordOutput calls to reduce log file size. All IO
   * inputs (Logger.processInputs) are always logged regardless of this flag.
   */
  public static final boolean MINIMAL_LOGGING = false;

  public static final double loopPeriodSecs = 0.02;
}
