package frc.robot.subsystems.objectdetection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.RobotController;

/**
 * ObjectDetection IO implementation that reads ball detections from the Jetson coprocessor via
 * NetworkTables structs.
 *
 * <p>The Jetson must publish a {@code BallDetection[]} array to the topic {@code
 * "<tableName>/balls"} using the matching struct schema.
 */
public class ObjectDetectionIOJetson implements ObjectDetectionIO {

  /** Maximum age of the last NT update before reporting disconnected (microseconds). */
  private static final long CONNECTION_TIMEOUT_US = 500_000L;

  private final StructArraySubscriber<BallDetection> ballsSubscriber;

  public ObjectDetectionIOJetson(String tableName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
    ballsSubscriber =
        table.getStructArrayTopic("balls", BallDetection.struct).subscribe(new BallDetection[0]);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    long lastUpdateUs = ballsSubscriber.getAtomic().serverTime;
    inputs.connected =
        lastUpdateUs > 0 && (RobotController.getFPGATime() - lastUpdateUs) < CONNECTION_TIMEOUT_US;

    BallDetection[] detections = ballsSubscriber.get();
    inputs.detectionX = new double[detections.length];
    inputs.detectionY = new double[detections.length];
    inputs.detectionDistance = new double[detections.length];
    for (int i = 0; i < detections.length; i++) {
      inputs.detectionX[i] = detections[i].x();
      inputs.detectionY[i] = detections[i].y();
      inputs.detectionDistance[i] = detections[i].distance();
    }
  }
}
