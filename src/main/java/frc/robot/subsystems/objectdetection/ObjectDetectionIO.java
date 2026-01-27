package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionInputs {
    public boolean connected = false;

    public FuelObservation[] targets = new FuelObservation[0];
  }

  public static record FuelObservation(Rotation2d pitch, Rotation2d yaw, double area, double timestamp) {}

  public default void updateInputs(ObjectDetectionInputs inputs) {}

  public default ObjectDetectionCameraSettings getCameraSettings() { return new ObjectDetectionCameraSettings(); }
}
