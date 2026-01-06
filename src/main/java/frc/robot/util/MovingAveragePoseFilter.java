package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;

public class MovingAveragePoseFilter {
  private Pose2d[] window;
  private int index = 0;

  public MovingAveragePoseFilter(int windowSize) {
    window = new Pose2d[windowSize];

    Arrays.fill(window, null);
  }

  public Pose2d calculate(Pose2d newPose) {
    window[index] = newPose;
    index = (index + 1) % window.length;

    double sumX = 0;
    double sumY = 0;
    double sumSinTheta = 0;
    double sumCosTheta = 0;
    int count = 0;

    for (Pose2d pose : window) {
      if (pose != null) {
        sumX += pose.getX();
        sumY += pose.getY();
        sumSinTheta += Math.sin(pose.getRotation().getRadians());
        sumCosTheta += Math.cos(pose.getRotation().getRadians());
        count++;
      }
    }

    if (count == 0) {
      return newPose; // Avoid division by zero
    }

    double avgX = sumX / count;
    double avgY = sumY / count;
    double avgTheta = Math.atan2(sumSinTheta / count, sumCosTheta / count);

    return new Pose2d(avgX, avgY, new Rotation2d(avgTheta));
  }
}
