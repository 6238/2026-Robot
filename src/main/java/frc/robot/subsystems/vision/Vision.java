// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Supplier<Pose2d> currentPoseEstimateSupplier;

  // Pre-allocated per-camera logging lists — cleared each cycle instead of reallocated
  private final List<Pose3d>[] tagPoseLists;
  private final List<Pose3d>[] acceptedPoseLists;
  private final List<Pose3d>[] rejectedPoseLists;

  // Pre-built log key strings — avoids string concatenation each cycle
  private final String[] tagPoseKeys;
  private final String[] acceptedKeys;
  private final String[] rejectedKeys;
  private final String[] cameraInputKeys;

  @SuppressWarnings("unchecked")
  public Vision(
      VisionConsumer consumer, Supplier<Pose2d> currentPoseEstimateSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.currentPoseEstimateSupplier = currentPoseEstimateSupplier;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    // Pre-allocate logging lists and cache key strings
    tagPoseLists = new ArrayList[io.length];
    acceptedPoseLists = new ArrayList[io.length];
    rejectedPoseLists = new ArrayList[io.length];
    tagPoseKeys = new String[io.length];
    acceptedKeys = new String[io.length];
    rejectedKeys = new String[io.length];
    cameraInputKeys = new String[io.length];
    for (int i = 0; i < io.length; i++) {
      tagPoseLists[i] = new ArrayList<>(16);
      acceptedPoseLists[i] = new ArrayList<>(8);
      rejectedPoseLists[i] = new ArrayList<>(8);
      tagPoseKeys[i] = "Vision/Camera" + i + "/TagPoses";
      acceptedKeys[i] = "Vision/Camera" + i + "/RobotPosesAccepted";
      rejectedKeys[i] = "Vision/Camera" + i + "/RobotPosesRejected";
      cameraInputKeys[i] = "Vision/Camera" + i;
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i], currentPoseEstimateSupplier.get());
      Logger.processInputs(cameraInputKeys[i], inputs[i]);
    }

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Clear logging lists (reuse existing allocations)
      List<Pose3d> tagPoses = tagPoseLists[cameraIndex];
      List<Pose3d> robotPosesAccepted = acceptedPoseLists[cameraIndex];
      List<Pose3d> robotPosesRejected = rejectedPoseLists[cameraIndex];
      tagPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate
                // || observation.averageTagDistance()
                //     > maxTagDistance // Must be within max tag distance
                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();
        // Must not be in an X exclusion zone
        // || isInXExclusionZone(observation.pose().getX());

        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        }
        robotPosesAccepted.add(observation.pose());

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      Logger.recordOutput(tagPoseKeys[cameraIndex], tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(acceptedKeys[cameraIndex], robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(rejectedKeys[cameraIndex], robotPosesRejected.toArray(new Pose3d[0]));
    }
  }

  private static boolean isInXExclusionZone(double x) {
    for (double[] zone : xExclusionZones) {
      if (x >= zone[0] && x <= zone[1]) return true;
    }
    return false;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
