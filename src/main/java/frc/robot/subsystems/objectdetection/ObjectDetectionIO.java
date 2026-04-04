package frc.robot.subsystems.objectdetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {

  @AutoLog
  public static class ObjectDetectionIOInputs {
    /** True when the Jetson is publishing data within the last 500 ms. */
    public boolean connected = false;

    /** Lateral offsets of each detected ball in camera frame (meters, right = positive). */
    public double[] detectionX = new double[0];

    /** Forward distances of each detected ball in camera frame (meters, always positive). */
    public double[] detectionY = new double[0];

    /** Euclidean distances to each detected ball (meters). */
    public double[] detectionDistance = new double[0];
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
