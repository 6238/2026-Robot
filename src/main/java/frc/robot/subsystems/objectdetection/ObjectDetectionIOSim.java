package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

/**
 * Simulation implementation of ObjectDetectionIO backed by MapleSim game-piece positions.
 *
 * <p>Each periodic cycle, all "Fuel" game pieces in the arena are queried. Those within the
 * camera's range and horizontal FOV are projected into camera frame and reported as detections,
 * faithfully simulating what a real Jetson-attached camera would see.
 *
 * <p>For unit tests that don't use the arena, call {@link #setDetections} to inject detections
 * directly.
 */
public class ObjectDetectionIOSim implements ObjectDetectionIO {

  /** Maximum detection range in metres. */
  private static final double MAX_RANGE_METERS = 5.0;

  /** Total horizontal field of view in radians (~62° — typical USB camera). */
  private static final double HORIZONTAL_FOV_RAD = Math.toRadians(62.0);

  private final Supplier<Pose2d> simPoseSupplier;

  /**
   * Transform from camera frame to robot frame (same convention as {@link ObjectDetection}). Used
   * to derive the camera's world pose from the simulated robot pose.
   */
  private final Transform2d cameraToRobot;

  /** Non-null only when {@link #setDetections} has been called to override arena queries. */
  private BallDetection[] manualDetections = null;

  /**
   * @param simPoseSupplier Ground-truth pose supplier — use {@code
   *     swerveDriveSimulation::getSimulatedDriveTrainPose} so the simulated camera sees the true
   *     world position, not the odometry estimate.
   * @param cameraToRobot Transform from camera frame to robot frame.
   */
  public ObjectDetectionIOSim(Supplier<Pose2d> simPoseSupplier, Transform2d cameraToRobot) {
    this.simPoseSupplier = simPoseSupplier;
    this.cameraToRobot = cameraToRobot;
  }

  /** Convenience constructor for a forward-facing camera mounted at the robot centre. */
  public ObjectDetectionIOSim(Supplier<Pose2d> simPoseSupplier) {
    this(simPoseSupplier, new Transform2d());
  }

  /**
   * Override arena queries with a fixed set of detections. Useful in unit tests that run without a
   * MapleSim arena. Pass no arguments (or call with an empty array) to clear the override.
   */
  public void setDetections(BallDetection... detections) {
    this.manualDetections = detections.length == 0 ? null : detections;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.connected = true;

    if (manualDetections != null) {
      inputs.detectionX = new double[manualDetections.length];
      inputs.detectionY = new double[manualDetections.length];
      inputs.detectionDistance = new double[manualDetections.length];
      for (int i = 0; i < manualDetections.length; i++) {
        inputs.detectionX[i] = manualDetections[i].x();
        inputs.detectionY[i] = manualDetections[i].y();
        inputs.detectionDistance[i] = manualDetections[i].distance();
      }
      return;
    }

    // ── MapleSim arena query ──────────────────────────────────────────────
    Pose2d robotPose = simPoseSupplier.get();

    // Camera pose in field frame: robot pose composed with the camera's offset/heading on the robot
    Pose2d cameraPose = robotPose.transformBy(cameraToRobot);

    Pose3d[] gamePieces = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");

    List<Double> xs = new ArrayList<>();
    List<Double> ys = new ArrayList<>();
    List<Double> dists = new ArrayList<>();

    for (Pose3d piece : gamePieces) {
      Translation2d pieceField = new Translation2d(piece.getX(), piece.getY());

      // Vector from camera to piece, rotated into camera frame
      // WPILib camera frame: x = forward, y = left  →  our convention: x = right, y = forward
      Translation2d delta = pieceField.minus(cameraPose.getTranslation());
      Translation2d inCamera = delta.rotateBy(cameraPose.getRotation().unaryMinus());

      double camY = inCamera.getX(); // WPILib x (forward) → our y (forward distance)
      double camX = -inCamera.getY(); // WPILib y (left)    → our x (right, negated)
      double dist = Math.hypot(camX, camY);

      // Must be in front of camera
      if (camY <= 0.05) continue;
      // Must be within range
      if (dist > MAX_RANGE_METERS) continue;
      // Must be within horizontal FOV
      if (Math.atan2(Math.abs(camX), camY) > HORIZONTAL_FOV_RAD / 2.0) continue;

      xs.add(camX);
      ys.add(camY);
      dists.add(dist);
    }

    inputs.detectionX = xs.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.detectionY = ys.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.detectionDistance = dists.stream().mapToDouble(Double::doubleValue).toArray();
  }
}
