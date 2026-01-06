package frc.robot.subsystems.objectdetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public TargetDetection[] targetDetections = new TargetDetection[0];
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
