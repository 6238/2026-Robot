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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentPoseEstimate) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        Pose3d fieldToRobotBest = Pose3d.kZero.plus(multitagResult.estimatedPose.best).relativeTo(aprilTagLayout.getOrigin()).plus(robotToCamera.inverse());

        double totalTagDistance;

        // Calculate average tag distance
        totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                fieldToRobotBest, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type
      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());

          Transform3d best_cameraToTarget = target.bestCameraToTarget;
          Transform3d best_fieldToCamera = fieldToTarget.plus(best_cameraToTarget.inverse());
          Transform3d best_fieldToRobot = best_fieldToCamera.plus(robotToCamera.inverse());
          Pose3d best_robotPose =
              new Pose3d(best_fieldToRobot.getTranslation(), best_fieldToRobot.getRotation());

          Transform3d alt_cameraToTarget = target.bestCameraToTarget;
          Transform3d alt_fieldToCamera = fieldToTarget.plus(alt_cameraToTarget.inverse());
          Transform3d alt_fieldToRobot = alt_fieldToCamera.plus(robotToCamera.inverse());
          Pose3d alt_robotPose =
              new Pose3d(alt_fieldToRobot.getTranslation(), alt_fieldToRobot.getRotation());

          double bestDistance =
              best_fieldToRobot
                  .getTranslation()
                  .getDistance(
                      new Translation3d(
                          currentPoseEstimate.getX(), currentPoseEstimate.getY(), 0.0));
          double worstDistance =
              alt_fieldToRobot
                  .getTranslation()
                  .getDistance(
                      new Translation3d(
                          currentPoseEstimate.getX(), currentPoseEstimate.getY(), 0.0));

          if (bestDistance < worstDistance) {
            // Add tag ID
            tagIds.add((short) target.fiducialId);

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    best_robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // Tag count
                    best_cameraToTarget.getTranslation().getNorm(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          } else {
            // Add tag ID
            tagIds.add((short) target.fiducialId);

            // Add observation
            poseObservations.add(
                new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    alt_robotPose, // 3D pose estimate
                    target.poseAmbiguity, // Ambiguity
                    1, // Tag count
                    alt_cameraToTarget.getTranslation().getNorm(), // Average tag distance
                    PoseObservationType.PHOTONVISION)); // Observation type
          }
        }
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
