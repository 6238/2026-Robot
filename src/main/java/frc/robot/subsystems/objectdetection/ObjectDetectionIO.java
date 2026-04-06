package frc.robot.subsystems.objectdetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {

  @AutoLog
  class ObjectDetectionIOInputs {
    public boolean connected = false;
    public long heartbeat = 0;
    public BallDetection[] camera0Detections = new BallDetection[0];
    public BallDetection[] camera1Detections = new BallDetection[0];
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
