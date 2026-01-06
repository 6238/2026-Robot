// package frc.robot.subsystems.objectdetection;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.wpilibj.Timer;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.Supplier;
// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

// public class ObjectDetectionIOSim implements ObjectDetectionIO {
//   private final double maxDetectionDistance;
//   private final double maxDetectionAngle;
//   private final double minConfidenceThreshold;
//   private final Supplier<Pose2d> poseSupplier;

//   /**
//    * Creates a simulated object detection IO using MapleSim game pieces
//    *
//    * @param maxDetectionDistance Maximum distance in meters the camera can detect objects
//    * @param maxDetectionAngle Maximum angle in radians from camera center to detect objects
//    */
//   public ObjectDetectionIOSim(
//       double maxDetectionDistance, double maxDetectionAngle, Supplier<Pose2d> poseSupplier) {
//     this.maxDetectionDistance = maxDetectionDistance;
//     this.maxDetectionAngle = maxDetectionAngle;
//     this.minConfidenceThreshold = 0.5;
//     this.poseSupplier = poseSupplier;
//   }

//   /**
//    * Creates a simulated object detection IO with default parameters
//    *
//    * @param gameObjectType The type of game piece to detect
//    */
//   public ObjectDetectionIOSim(Supplier<Pose2d> poseSupplier) {
//     this(5.0, Math.toRadians(90.0), poseSupplier);
//   }

//   @Override
//   public void updateInputs(ObjectDetectionIOInputs inputs) {
//     inputs.connected = true;

//     Pose2d robotPose = poseSupplier.get();
//     Pose2d cameraPose = robotPose.transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

//     GamePieceOnFieldSimulation[] gamePieces =
//
// SimulatedArena.getInstance().gamePiecesOnField().toArray(GamePieceOnFieldSimulation[]::new);

//     List<TargetObservation> observations = new ArrayList<>();

//     for (GamePieceOnFieldSimulation gamePiece : gamePieces) {
//       Pose2d gamePiece2d =
//           new Pose2d(
//               gamePiece.getPoseOnField().getX(),
//               gamePiece.getPoseOnField().getY(),
//               new Rotation2d());

//       Transform2d cameraToGamePiece = new Transform2d(cameraPose, gamePiece2d);

//       double dx = cameraToGamePiece.getX();
//       double dy = cameraToGamePiece.getY();
//       double distance = Math.hypot(dx, dy);
//       double angle = Math.atan2(dy, dx);

//       if (distance > maxDetectionDistance) {
//         continue;
//       }

//       if (Math.abs(angle) > maxDetectionAngle / 2.0) {
//         continue;
//       }

//       double area = 1000.0 / (distance * distance + 0.1); // TODO: Realistic Area for gamepieces

//       double distanceFactor = 1.0 - (distance / maxDetectionDistance);
//       double angleFactor = 1.0 - (Math.abs(angle) / (maxDetectionAngle / 2.0));
//       double confidence = (distanceFactor * 0.7 + angleFactor * 0.3);

//       confidence *= (0.95 + Math.random() * 0.05);

//       if (confidence >= minConfidenceThreshold) {
//         observations.add(new TargetObservation(dx, dy, area, confidence, Timer.getTimestamp()));
//       }
//     }

//     inputs.targetObservations = observations.toArray(new TargetObservation[0]);
//   }
// }
