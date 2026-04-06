package frc.robot.subsystems.objectdetection;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class ObjectDetectionConstants {

  // Camera mount offsets from robot center (robot frame: x=forward, y=left)
  // TODO: fill in from robot CAD
  public static final Transform2d CAMERA_0_ROBOT_OFFSET =
      new Transform2d(0.0, 0.15, Rotation2d.fromDegrees(0));
  public static final Transform2d CAMERA_1_ROBOT_OFFSET =
      new Transform2d(0.0, -0.15, Rotation2d.fromDegrees(0));

  // Rolling buffer
  public static final double BUFFER_WINDOW_SECS = 1.5;

  // DBSCAN
  public static final double DBSCAN_EPSILON = 0.8; // meters
  public static final int DBSCAN_MIN_PTS = 2;

  // Cluster scoring
  public static final double COUNT_SCALAR = 1.0;
  public static final double DIST_SCALAR = 0.5;
  public static final double DECAY_K = 2.0; // exponential decay per second

  // Replan gate
  public static final double REPLAN_THRESHOLD_METERS = 0.15;

  // PathPlanner constraints
  public static final double MAX_COLLECTION_SPEED_MPS = 2.0;
  public static final double MAX_COLLECTION_ACCEL_MPS2 = 3.0;
  public static final double MAX_COLLECTION_ANG_SPEED_RAD = 2 * Math.PI;
  public static final double MAX_COLLECTION_ANG_ACCEL_RAD = 4 * Math.PI;
  public static final PathConstraints COLLECTION_CONSTRAINTS =
      new PathConstraints(
          MAX_COLLECTION_SPEED_MPS,
          MAX_COLLECTION_ACCEL_MPS2,
          MAX_COLLECTION_ANG_SPEED_RAD,
          MAX_COLLECTION_ANG_ACCEL_RAD);

  // Minimum field speed (m/s) before overriding holonomic rotation toward intake-leading
  public static final double ROTATION_OVERRIDE_MIN_SPEED = 0.15;

  private ObjectDetectionConstants() {}
}
