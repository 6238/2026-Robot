package frc.robot.subsystems.objectdetection;

import static frc.robot.subsystems.objectdetection.ObjectDetectionAlgorithms.TimestampedFieldDetection;
import static frc.robot.subsystems.objectdetection.ObjectDetectionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {

  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();
  private final Drive drive;

  private final ArrayDeque<TimestampedFieldDetection> buffer = new ArrayDeque<>();
  private long lastHeartbeat = -1;
  private boolean committed = false;
  private Optional<PathPlannerPath> bestPath = Optional.empty();
  private Translation2d lastClusterCentroid = null;

  public ObjectDetection(ObjectDetectionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);

    boolean newData = inputs.heartbeat != lastHeartbeat;

    if (newData) {
      lastHeartbeat = inputs.heartbeat;
      Pose2d robotPose = drive.getPose();
      double now = Timer.getTimestamp();
      appendDetections(inputs.camera0Detections, CAMERA_0_ROBOT_OFFSET, robotPose, now);
      appendDetections(inputs.camera1Detections, CAMERA_1_ROBOT_OFFSET, robotPose, now);
    }

    double cutoff = Timer.getTimestamp() - BUFFER_WINDOW_SECS;
    while (!buffer.isEmpty() && buffer.peekFirst().timestamp() < cutoff) {
      buffer.pollFirst();
    }

    if (newData && !committed) {
      runPipeline();
    }

    Logger.recordOutput("ObjectDetection/BufferSize", buffer.size());
    Logger.recordOutput("ObjectDetection/Committed", committed);
    Logger.recordOutput("ObjectDetection/HasPath", bestPath.isPresent());
  }

  private void appendDetections(
      BallDetection[] detections, Transform2d cameraMount, Pose2d robotPose, double timestamp) {
    for (BallDetection d : detections) {
      // Camera local (x=right, y=forward) → robot NWU (x=forward, y=left)
      Translation2d camNWU = new Translation2d(d.y(), -d.x());
      // Rotate by mount yaw, add mount translation
      Translation2d robotRel =
          camNWU.rotateBy(cameraMount.getRotation()).plus(cameraMount.getTranslation());
      // Transform to field frame
      Translation2d fieldPos =
          robotRel.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
      buffer.addLast(new TimestampedFieldDetection(fieldPos.getX(), fieldPos.getY(), timestamp));
    }
  }

  private void runPipeline() {
    List<TimestampedFieldDetection> snapshot = new ArrayList<>(buffer);
    if (snapshot.size() < 2) return;

    List<List<TimestampedFieldDetection>> clusters =
        ObjectDetectionAlgorithms.dbscan(snapshot, DBSCAN_EPSILON, DBSCAN_MIN_PTS);
    if (clusters.isEmpty()) return;

    Logger.recordOutput("ObjectDetection/ClusterCount", clusters.size());

    double now = Timer.getTimestamp();
    Translation2d robotPos = drive.getPose().getTranslation();

    double bestScore = Double.NEGATIVE_INFINITY;
    List<TimestampedFieldDetection> bestCluster = null;
    for (List<TimestampedFieldDetection> cluster : clusters) {
      double score =
          ObjectDetectionAlgorithms.scoreCluster(
              cluster, robotPos, now, DECAY_K, COUNT_SCALAR, DIST_SCALAR);
      if (score > bestScore) {
        bestScore = score;
        bestCluster = cluster;
      }
    }
    Logger.recordOutput("ObjectDetection/BestClusterScore", bestScore);

    if (bestCluster == null || bestCluster.size() < 2) return;

    Translation2d newCentroid = ObjectDetectionAlgorithms.clusterCentroid(bestCluster);

    if (lastClusterCentroid != null
        && newCentroid.getDistance(lastClusterCentroid) < REPLAN_THRESHOLD_METERS) {
      return;
    }
    lastClusterCentroid = newCentroid;

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(bestCluster);

    Pose2d[] collectionWps = ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, robotPos);

    Translation2d[] wpTranslations = new Translation2d[4];
    for (int i = 0; i < 4; i++) wpTranslations[i] = collectionWps[i].getTranslation();
    Logger.recordOutput("ObjectDetection/ClusterWaypoints", wpTranslations);

    Translation2d[] allPoints = new Translation2d[snapshot.size()];
    for (int i = 0; i < snapshot.size(); i++)
      allPoints[i] = new Translation2d(snapshot.get(i).x(), snapshot.get(i).y());
    Logger.recordOutput("ObjectDetection/AllBufferedDetections", allPoints);

    Pose2d robotPose = drive.getPose();
    Translation2d toFirst = collectionWps[0].getTranslation().minus(robotPose.getTranslation());
    Rotation2d approachDir =
        (toFirst.getNorm() > 0.05)
            ? new Rotation2d(toFirst.getX(), toFirst.getY())
            : collectionWps[0].getRotation();
    Pose2d startPose = new Pose2d(robotPose.getTranslation(), approachDir);

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            startPose, collectionWps[0], collectionWps[1], collectionWps[2], collectionWps[3]);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            COLLECTION_CONSTRAINTS,
            null,
            new GoalEndState(0.0, collectionWps[3].getRotation()));
    path.preventFlipping = true;

    bestPath = Optional.of(path);
  }

  /**
   * Continuously collects the best cluster, then chains to the next best, repeating until
   * cancelled. Overrides holonomic rotation so the intake (back of robot) faces the direction of
   * travel.
   */
  public Command continuousCollectionCommand() {
    DoubleSupplier intakeLeadingRotation =
        () -> {
          ChassisSpeeds field =
              ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
          double vx = field.vxMetersPerSecond;
          double vy = field.vyMetersPerSecond;
          if (Math.hypot(vx, vy) > ROTATION_OVERRIDE_MIN_SPEED) {
            return Math.atan2(vy, vx) + Math.PI;
          }
          return drive.getRotation().getRadians();
        };

    return Commands.sequence(
        Commands.waitUntil(() -> bestPath.isPresent()),
        Commands.sequence(
                Commands.defer(
                    () ->
                        AutoBuilder.followPath(bestPath.get())
                            .beforeStarting(
                                () -> {
                                  setCommitted(true);
                                  PPHolonomicDriveController.overrideRotationFeedback(
                                      intakeLeadingRotation);
                                })
                            .finallyDo(
                                interrupted -> {
                                  setCommitted(false);
                                  PPHolonomicDriveController.clearRotationFeedbackOverride();
                                }),
                    Set.of(drive)),
                Commands.waitUntil(() -> bestPath.isPresent()))
            .repeatedly());
  }

  public void setCommitted(boolean committed) {
    this.committed = committed;
    if (!committed) {
      bestPath = Optional.empty();
      lastClusterCentroid = null;
    }
  }

  public boolean isCommitted() {
    return committed;
  }

  public Optional<PathPlannerPath> getBestPath() {
    return bestPath;
  }

  /** Exposed for testing only. */
  public int getBufferSize() {
    return buffer.size();
  }
}
