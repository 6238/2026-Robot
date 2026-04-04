package frc.robot.subsystems.objectdetection;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that consumes ball-position data from the Jetson coprocessor, accumulates a 0.5-second
 * history of field-relative ball positions, clusters them with DBSCAN, and can generate a
 * PathPlanner path through the cluster using the Binned-Centroid PCA method.
 *
 * <h3>Pipeline</h3>
 *
 * <ol>
 *   <li>Receive raw camera-frame detections from {@link ObjectDetectionIO}.
 *   <li>Transform to field frame via camera→robot→field transforms.
 *   <li>Expire points older than {@value #POINT_LIFESPAN_SECONDS} s.
 *   <li>Run DBSCAN (ε={@value #DBSCAN_EPSILON} m, minPts={@value #DBSCAN_MIN_POINTS}).
 *   <li>On demand: PCA + binned centroids → PathPlanner path.
 * </ol>
 */
public class ObjectDetection extends SubsystemBase {

  // ── Point-history lifespan ────────────────────────────────────────────────
  private static final double POINT_LIFESPAN_SECONDS = 0.5;

  // ── DBSCAN parameters ─────────────────────────────────────────────────────
  /** Neighbourhood radius in metres. */
  private static final double DBSCAN_EPSILON = 0.5;

  /** Minimum neighbours (including self) to be considered a core point. */
  private static final int DBSCAN_MIN_POINTS = 2;

  // ── Binned-centroid parameters ────────────────────────────────────────────
  private static final int BIN_COUNT = 5;

  // ── PathPlanner constraints ───────────────────────────────────────────────
  /** Constraints used while following the generated ball path. */
  private static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(2.0, 2.5, Math.toRadians(360), Math.toRadians(540));

  /** Constraints used while pathfinding to the start of the ball path. */
  private static final PathConstraints PATHFIND_CONSTRAINTS =
      new PathConstraints(3.5, 3.0, Math.toRadians(360), Math.toRadians(540));

  // ── Internal record for timestamped field-frame points ───────────────────
  private record TimestampedPoint(Translation2d point, double timestamp) {}

  /**
   * How far the tracked cluster centroid must shift (metres) before the active path is abandoned
   * and a new one is planned.
   */
  private static final double REPLAN_THRESHOLD_METERS = 0.35;

  // ── State ─────────────────────────────────────────────────────────────────
  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Transform from camera frame to robot frame. Default is identity (camera at robot centre facing
   * forward).
   */
  private final Transform2d cameraToRobot;

  private final List<TimestampedPoint> pointHistory = new ArrayList<>();
  private List<List<Translation2d>> currentClusters = List.of();

  /** Centroid of the cluster that was used to build the currently-executing path. */
  private Translation2d committedCentroid = null;

  // ─────────────────────────────────────────────────────────────────────────

  /**
   * @param io IO layer (Jetson or sim).
   * @param poseSupplier Supplier of the robot's current field-relative pose (from Drive).
   * @param cameraToRobot Transform from camera frame to robot frame. Pass {@code new Transform2d()}
   *     for a forward-facing camera at the robot centre.
   */
  public ObjectDetection(
      ObjectDetectionIO io, Supplier<Pose2d> poseSupplier, Transform2d cameraToRobot) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.cameraToRobot = cameraToRobot;
  }

  /** Convenience constructor for a forward-facing camera mounted at the robot centre. */
  public ObjectDetection(ObjectDetectionIO io, Supplier<Pose2d> poseSupplier) {
    this(io, poseSupplier, new Transform2d());
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);

    double now = Timer.getTimestamp();
    Pose2d robotPose = poseSupplier.get();

    // Ingest new detections into field-frame history
    for (int i = 0; i < inputs.detectionX.length; i++) {
      Translation2d ballInField =
          cameraToField(inputs.detectionX[i], inputs.detectionY[i], robotPose);
      pointHistory.add(new TimestampedPoint(ballInField, now));
    }

    // Expire stale points
    pointHistory.removeIf(p -> now - p.timestamp() > POINT_LIFESPAN_SECONDS);

    // Cluster
    List<Translation2d> allPoints = pointHistory.stream().map(TimestampedPoint::point).toList();
    currentClusters = dbscan(allPoints);

    // Logging
    Logger.recordOutput("ObjectDetection/PointCount", allPoints.size());
    Logger.recordOutput("ObjectDetection/ClusterCount", currentClusters.size());
    Logger.recordOutput(
        "ObjectDetection/AllClusterPoints",
        currentClusters.stream().flatMap(List::stream).toArray(Translation2d[]::new));
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /** Returns all current DBSCAN clusters (may be empty). */
  public List<List<Translation2d>> getClusters() {
    return Collections.unmodifiableList(currentClusters);
  }

  /** Returns the largest cluster by point count, or empty if no clusters exist. */
  public Optional<List<Translation2d>> getLargestCluster() {
    return currentClusters.stream().max(Comparator.comparingInt(List::size));
  }

  /**
   * Returns true when the largest cluster has shifted more than {@value #REPLAN_THRESHOLD_METERS} m
   * from the centroid that was locked in when the active path started. Used as the {@code until()}
   * condition inside {@link #continuousBallIntakeCommand}.
   */
  public boolean shouldReplan() {
    if (committedCentroid == null) return false;
    return getLargestCluster()
        .map(ObjectDetection::clusterCentroid)
        .map(c -> c.getDistance(committedCentroid) > REPLAN_THRESHOLD_METERS)
        .orElse(false);
  }

  /**
   * Returns a command that continuously chases balls using the detected cluster:
   *
   * <ol>
   *   <li>Snapshot the current cluster centroid as the "committed" reference.
   *   <li>Compute and follow the binned-centroid path ({@link #createBallPathCommand}).
   *   <li>If the cluster shifts by more than {@value #REPLAN_THRESHOLD_METERS} m mid-path, abandon
   *       the path and immediately replan from the updated cluster.
   *   <li>If the path finishes naturally (robot reached the end), immediately replan with any
   *       remaining / newly detected balls.
   * </ol>
   *
   * <p>The loop runs until the caller cancels it (e.g. by releasing the button).
   *
   * @param driveRequirement The Drive subsystem passed through to PathPlanner commands.
   */
  public Command continuousBallIntakeCommand(SubsystemBase driveRequirement) {
    return Commands.sequence(
            // Lock in the current cluster centroid so shouldReplan() has a reference point.
            Commands.runOnce(
                () ->
                    committedCentroid =
                        getLargestCluster().map(ObjectDetection::clusterCentroid).orElse(null)),
            // Follow the path; cut it short if the cluster moves significantly.
            createBallPathCommand(driveRequirement).until(this::shouldReplan))
        .repeatedly();
  }

  /**
   * Returns a command that pathfinds to the near end of the detected ball path, then follows the
   * full binned-centroid spine through the cluster.
   *
   * <p>The command is deferred — the path is (re)computed from the live cluster at the moment the
   * command is scheduled, not when this method is called.
   *
   * @param driveRequirement The Drive subsystem, so the scheduler can manage conflicts.
   * @return Deferred pathfind+follow command, or {@code Commands.none()} if no usable cluster.
   */
  public Command createBallPathCommand(SubsystemBase driveRequirement) {
    return Commands.defer(
        () -> {
          Optional<List<Translation2d>> clusterOpt = getLargestCluster();
          if (clusterOpt.isEmpty() || clusterOpt.get().size() < 2) {
            return Commands.none();
          }

          List<Translation2d> centroids = binnedCentroids(clusterOpt.get(), BIN_COUNT);
          if (centroids.size() < 2) {
            return Commands.none();
          }

          // Orient path so we approach the nearer end first
          Translation2d robotPos = poseSupplier.get().getTranslation();
          if (robotPos.getDistance(centroids.get(centroids.size() - 1))
              < robotPos.getDistance(centroids.get(0))) {
            Collections.reverse(centroids);
          }

          List<Pose2d> waypointPoses = centroidsToWaypoints(centroids);

          List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(waypointPoses);
          PathPlannerPath path =
              new PathPlannerPath(
                  waypoints,
                  PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0.0, waypointPoses.get(waypointPoses.size() - 1).getRotation()));
          path.preventFlipping = true;

          return AutoBuilder.pathfindThenFollowPath(path, PATHFIND_CONSTRAINTS);
        },
        Set.of(driveRequirement));
  }

  // ── Geometry helpers ──────────────────────────────────────────────────────

  /**
   * Converts a raw camera-frame detection to a field-relative Translation2d.
   *
   * <p>Camera frame: x = right, y = forward. WPILib robot frame: x = forward, y = left.
   */
  private Translation2d cameraToField(double camX, double camY, Pose2d robotPose) {
    // Camera → robot frame
    Translation2d inCamera = new Translation2d(camY, -camX); // forward, left
    Translation2d inRobot =
        inCamera.rotateBy(cameraToRobot.getRotation()).plus(cameraToRobot.getTranslation());
    // Robot → field frame
    return inRobot.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  }

  // ── DBSCAN ────────────────────────────────────────────────────────────────

  private List<List<Translation2d>> dbscan(List<Translation2d> points) {
    int n = points.size();
    if (n == 0) return List.of();

    // 0 = unvisited, -1 = noise, >0 = cluster ID
    int[] labels = new int[n];
    int nextCluster = 0;

    for (int i = 0; i < n; i++) {
      if (labels[i] != 0) continue;
      List<Integer> neighbors = regionQuery(points, i);
      if (neighbors.size() < DBSCAN_MIN_POINTS) {
        labels[i] = -1;
      } else {
        nextCluster++;
        labels[i] = nextCluster;
        expandCluster(points, labels, neighbors, nextCluster);
      }
    }

    Map<Integer, List<Translation2d>> map = new HashMap<>();
    for (int i = 0; i < n; i++) {
      if (labels[i] > 0) {
        map.computeIfAbsent(labels[i], k -> new ArrayList<>()).add(points.get(i));
      }
    }
    return new ArrayList<>(map.values());
  }

  private List<Integer> regionQuery(List<Translation2d> points, int idx) {
    Translation2d p = points.get(idx);
    List<Integer> neighbors = new ArrayList<>();
    for (int i = 0; i < points.size(); i++) {
      if (p.getDistance(points.get(i)) <= DBSCAN_EPSILON) {
        neighbors.add(i);
      }
    }
    return neighbors;
  }

  private void expandCluster(
      List<Translation2d> points, int[] labels, List<Integer> seeds, int clusterId) {
    ArrayDeque<Integer> queue = new ArrayDeque<>();
    for (int s : seeds) {
      if (labels[s] == 0) {
        labels[s] = clusterId;
        queue.add(s);
      } else if (labels[s] == -1) {
        labels[s] = clusterId; // noise → border point
      }
    }

    while (!queue.isEmpty()) {
      int q = queue.poll();
      List<Integer> qNeighbors = regionQuery(points, q);
      if (qNeighbors.size() >= DBSCAN_MIN_POINTS) {
        for (int r : qNeighbors) {
          if (labels[r] == 0) {
            labels[r] = clusterId;
            queue.add(r);
          } else if (labels[r] == -1) {
            labels[r] = clusterId;
          }
        }
      }
    }
  }

  // ── Binned-centroid PCA ───────────────────────────────────────────────────

  /**
   * Projects cluster points onto their PCA major axis, divides into {@code numBins} equal segments,
   * and returns the centroid of each non-empty bin in order along the axis.
   */
  private List<Translation2d> binnedCentroids(List<Translation2d> points, int numBins) {
    int n = points.size();

    // 1. Mean
    double mx = 0, my = 0;
    for (Translation2d p : points) {
      mx += p.getX();
      my += p.getY();
    }
    mx /= n;
    my /= n;

    // 2. Covariance matrix (2×2 symmetric)
    double cxx = 0, cxy = 0, cyy = 0;
    for (Translation2d p : points) {
      double dx = p.getX() - mx;
      double dy = p.getY() - my;
      cxx += dx * dx;
      cxy += dx * dy;
      cyy += dy * dy;
    }
    cxx /= n;
    cxy /= n;
    cyy /= n;

    // 3. Major eigenvector of the 2×2 symmetric covariance matrix
    //    λ = (trace/2) ± sqrt((trace/2)² - det)
    double half = (cxx + cyy) / 2.0;
    double det = cxx * cyy - cxy * cxy;
    double gap = Math.sqrt(Math.max(0.0, half * half - det));
    double lambda1 = half + gap; // larger eigenvalue

    double ex, ey;
    if (Math.abs(cxy) > 1e-9) {
      ex = cxy;
      ey = lambda1 - cxx;
    } else {
      // Diagonal covariance — pick the axis with larger variance
      ex = (cxx >= cyy) ? 1.0 : 0.0;
      ey = (cxx >= cyy) ? 0.0 : 1.0;
    }
    double mag = Math.hypot(ex, ey);
    ex /= mag;
    ey /= mag;

    // 4. Project each point onto the major axis
    double[] proj = new double[n];
    double minProj = Double.MAX_VALUE;
    double maxProj = -Double.MAX_VALUE;
    for (int i = 0; i < n; i++) {
      proj[i] = (points.get(i).getX() - mx) * ex + (points.get(i).getY() - my) * ey;
      if (proj[i] < minProj) minProj = proj[i];
      if (proj[i] > maxProj) maxProj = proj[i];
    }

    double range = maxProj - minProj;
    if (range < 1e-6) {
      // Degenerate cluster — return its centroid as a single waypoint
      return List.of(new Translation2d(mx, my));
    }

    // 5. Bin + centroid
    double binSize = range / numBins;
    List<Translation2d> centroids = new ArrayList<>();
    for (int b = 0; b < numBins; b++) {
      double lo = minProj + b * binSize;
      double hi = lo + binSize;
      double sumX = 0, sumY = 0;
      int count = 0;
      for (int i = 0; i < n; i++) {
        // Include the last-bin boundary to avoid floating-point off-by-one
        boolean inBin = (proj[i] >= lo) && (proj[i] < hi || b == numBins - 1);
        if (inBin) {
          sumX += points.get(i).getX();
          sumY += points.get(i).getY();
          count++;
        }
      }
      if (count > 0) {
        centroids.add(new Translation2d(sumX / count, sumY / count));
      }
    }
    return centroids;
  }

  /**
   * Converts an ordered centroid list to Pose2d waypoints. Each pose faces toward the next centroid
   * (or continues the direction of the previous segment for the last point).
   */
  private List<Pose2d> centroidsToWaypoints(List<Translation2d> centroids) {
    List<Pose2d> poses = new ArrayList<>();
    for (int i = 0; i < centroids.size(); i++) {
      Translation2d cur = centroids.get(i);
      Translation2d ref = (i < centroids.size() - 1) ? centroids.get(i + 1) : centroids.get(i - 1);
      double angle = Math.atan2(ref.getY() - cur.getY(), ref.getX() - cur.getX());
      // Last point: keep the same heading direction (don't flip it)
      if (i == centroids.size() - 1) {
        angle = poses.get(poses.size() - 1).getRotation().getRadians();
      }
      poses.add(new Pose2d(cur, new Rotation2d(angle)));
    }
    return poses;
  }

  /** Returns the mean position of a cluster. */
  private static Translation2d clusterCentroid(List<Translation2d> cluster) {
    double x = cluster.stream().mapToDouble(Translation2d::getX).average().orElse(0);
    double y = cluster.stream().mapToDouble(Translation2d::getY).average().orElse(0);
    return new Translation2d(x, y);
  }
}
