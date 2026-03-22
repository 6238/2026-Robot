package subsystems;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import org.junit.jupiter.api.Test;

/**
 * Tests that VisionIOPhotonVision correctly converts a field-to-camera transform (multitag PNP
 * result) into the robot pose in field coordinates by applying the inverse of the robot-to-camera
 * transform.
 */
class VisionIOPhotonVisionTest {
  static final double DELTA = 1e-9;

  // ── camera at field origin ──────────────────────────────────────────────

  @Test
  void cameraAtOrigin_noRotation_robotPoseIsInverseOfCameraTransform() {
    // Camera is offset from robot center (translation only, no rotation)
    Transform3d robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.3), new Rotation3d());
    // Multitag reports camera at field origin with no rotation (identity transform)
    Transform3d fieldToCamera = new Transform3d();

    Pose3d result = VisionIOPhotonVision.computeRobotPose(fieldToCamera, robotToCamera);

    // Robot center must sit at the inverse of the camera offset from the field origin
    Pose3d expected =
        Pose3d.kZero.relativeTo(aprilTagLayout.getOrigin()).plus(robotToCamera.inverse());
    assertPose3dEquals(expected, result);
  }

  @Test
  void cameraAtOrigin_withRotation_robotPoseIsInverseOfCameraTransform() {
    // Camera is offset and rotated relative to robot center
    Transform3d robotToCamera =
        new Transform3d(new Translation3d(0.3, -0.2, 0.4), new Rotation3d(0.1, 0.2, 1.5));
    // Multitag reports camera at field origin
    Transform3d fieldToCamera = new Transform3d();

    Pose3d result = VisionIOPhotonVision.computeRobotPose(fieldToCamera, robotToCamera);

    Pose3d expected =
        Pose3d.kZero.relativeTo(aprilTagLayout.getOrigin()).plus(robotToCamera.inverse());
    assertPose3dEquals(expected, result);
  }

  // ── camera at known field position ─────────────────────────────────────

  @Test
  void cameraForwardOfRobot_cameraAtKnownField_robotCenterBehindCamera() {
    // Camera is 0.5 m forward of robot center along X, no rotation
    Transform3d robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.0), new Rotation3d());
    // Multitag places camera at X = 1.0 in field coordinates
    Transform3d fieldToCamera = new Transform3d(new Translation3d(1.0, 0.0, 0.0), new Rotation3d());

    Pose3d result = VisionIOPhotonVision.computeRobotPose(fieldToCamera, robotToCamera);

    // Robot center is 0.5 m behind camera → field X = 0.5
    assertEquals(0.5, result.getX(), DELTA);
    assertEquals(0.0, result.getY(), DELTA);
    assertEquals(0.0, result.getZ(), DELTA);
  }

  @Test
  void cameraSideOffsetLeft_cameraAtKnownField_robotCenterShiftedRight() {
    // Camera is 0.2 m to the left of robot center (+Y in robot frame), no rotation
    Transform3d robotToCamera = new Transform3d(new Translation3d(0.0, 0.2, 0.0), new Rotation3d());
    // Multitag places camera at Y = 1.0 in field coordinates
    Transform3d fieldToCamera = new Transform3d(new Translation3d(0.0, 1.0, 0.0), new Rotation3d());

    Pose3d result = VisionIOPhotonVision.computeRobotPose(fieldToCamera, robotToCamera);

    // Robot center is 0.2 m to the right of camera → field Y = 0.8
    assertEquals(0.0, result.getX(), DELTA);
    assertEquals(0.8, result.getY(), DELTA);
    assertEquals(0.0, result.getZ(), DELTA);
  }

  // ── helper ─────────────────────────────────────────────────────────────

  private void assertPose3dEquals(Pose3d expected, Pose3d actual) {
    assertEquals(expected.getX(), actual.getX(), DELTA, "X");
    assertEquals(expected.getY(), actual.getY(), DELTA, "Y");
    assertEquals(expected.getZ(), actual.getZ(), DELTA, "Z");
    assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), DELTA, "roll");
    assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), DELTA, "pitch");
    assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), DELTA, "yaw");
  }
}
