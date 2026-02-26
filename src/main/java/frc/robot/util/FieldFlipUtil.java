package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public final class FieldFlipUtil {
  public static final double FIELD_LENGTH_METERS = 16.54;
  public static final double FIELD_WIDTH_METERS = 8.21;

  private FieldFlipUtil() {}

  public static Translation2d flipVerticalMidline(Translation2d original) {
    double flippedX = FIELD_LENGTH_METERS - original.getX(); // [web:4]
    return new Translation2d(flippedX, original.getY());
  }

  public static Translation2d flipHorizontalMidline(Translation2d original) {
    double flippedY = FIELD_WIDTH_METERS - original.getY(); // [web:4]
    return new Translation2d(original.getX(), flippedY);
  }

  public static Translation2d flipBothMidlines(Translation2d original) {
    double flippedX = FIELD_LENGTH_METERS - original.getX();
    double flippedY = FIELD_WIDTH_METERS - original.getY();
    return new Translation2d(flippedX, flippedY);
  }
}
