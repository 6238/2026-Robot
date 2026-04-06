package frc.robot.subsystems.objectdetection;

public class ObjectDetectionIOSim implements ObjectDetectionIO {

  private BallDetection[] camera0 = new BallDetection[0];
  private BallDetection[] camera1 = new BallDetection[0];
  private long simulatedHeartbeat = 0;

  /** Inject synthetic detections from simulation code. Increments the heartbeat automatically. */
  public void setDetections(BallDetection[] camera0Detections, BallDetection[] camera1Detections) {
    this.camera0 = camera0Detections;
    this.camera1 = camera1Detections;
    simulatedHeartbeat++;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.camera0Detections = camera0;
    inputs.camera1Detections = camera1;
    inputs.heartbeat = simulatedHeartbeat;
    inputs.connected = true;
  }
}
