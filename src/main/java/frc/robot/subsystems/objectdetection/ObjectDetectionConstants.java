package frc.robot.subsystems.objectdetection;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ObjectDetectionConstants {
  public static final String JetsonTable = "ObjectDetection";

  public static final double HEARTBEAT_TOLERANCE = 0.5;
  public static final int UNCONNECTED_HEARTBEAT_VALUE = -1;

  public static final Transform2d ROBOT_TO_CAMERA =
      new Transform2d(new Translation2d(Inches.of(15).abs(Meters), 0), new Rotation2d(0.0));

  public static final int POSE_FILTER_TAPS = 5;

  public static final Time OBJECT_HISTORY_TIMEOUT = Seconds.of(1);
  public static final Distance OBJECT_ASSOCIATION_DISTANCE = Meters.of(1.2);
}
