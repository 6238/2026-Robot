package subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.objectdetection.BallDetection;
import frc.robot.subsystems.objectdetection.ObjectDetection;
import frc.robot.subsystems.objectdetection.ObjectDetectionIO;
import frc.robot.subsystems.objectdetection.ObjectDetectionIOInputsAutoLogged;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ObjectDetectionTest {

  private ObjectDetectionIO io;
  private Drive drive;
  private ObjectDetection subsystem;

  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setup() {
    io = mock(ObjectDetectionIO.class);
    drive = mock(Drive.class);
    when(drive.getPose()).thenReturn(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    subsystem = new ObjectDetection(io, drive);

    doAnswer(
            inv -> {
              ObjectDetectionIOInputsAutoLogged inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.heartbeat = 0;
              inputs.camera0Detections = new BallDetection[0];
              inputs.camera1Detections = new BallDetection[0];
              return null;
            })
        .when(io)
        .updateInputs(any());
  }

  @Test
  void periodic_newHeartbeat_addsDetectionsToBuffer() {
    doAnswer(
            inv -> {
              ObjectDetectionIOInputsAutoLogged inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.heartbeat = 1;
              inputs.camera0Detections = new BallDetection[] {new BallDetection(0.0, 2.0, 2.0)};
              inputs.camera1Detections = new BallDetection[0];
              return null;
            })
        .when(io)
        .updateInputs(any());

    subsystem.periodic();

    assertEquals(1, subsystem.getBufferSize());
  }

  @Test
  void periodic_sameHeartbeat_doesNotAddDuplicates() {
    doAnswer(
            inv -> {
              ObjectDetectionIOInputsAutoLogged inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.heartbeat = 1;
              inputs.camera0Detections = new BallDetection[] {new BallDetection(0.0, 2.0, 2.0)};
              inputs.camera1Detections = new BallDetection[0];
              return null;
            })
        .when(io)
        .updateInputs(any());

    subsystem.periodic();
    subsystem.periodic();

    assertEquals(1, subsystem.getBufferSize());
  }

  @Test
  void setCommitted_false_clearsBestPath() {
    subsystem.setCommitted(true);
    subsystem.setCommitted(false);
    assertTrue(subsystem.getBestPath().isEmpty());
  }

  @Test
  void periodic_multipleDetectionsInOneCluster_generatesBestPath() {
    final long[] heartbeat = {0};
    doAnswer(
            inv -> {
              ObjectDetectionIOInputsAutoLogged inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.heartbeat = ++heartbeat[0];
              inputs.camera0Detections =
                  new BallDetection[] {
                    new BallDetection(0.0, 2.0, 2.0),
                    new BallDetection(0.1, 2.1, 2.1),
                    new BallDetection(-0.1, 2.2, 2.2),
                    new BallDetection(0.0, 2.3, 2.3)
                  };
              inputs.camera1Detections = new BallDetection[0];
              return null;
            })
        .when(io)
        .updateInputs(any());

    subsystem.periodic();

    assertTrue(
        subsystem.getBestPath().isPresent(), "Should generate a path when a cluster is found");
  }

  @Test
  void periodic_committed_doesNotReplan() {
    final long[] heartbeat = {0};
    doAnswer(
            inv -> {
              ObjectDetectionIOInputsAutoLogged inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.heartbeat = ++heartbeat[0];
              inputs.camera0Detections =
                  new BallDetection[] {
                    new BallDetection(0.0, 2.0, 2.0),
                    new BallDetection(0.1, 2.1, 2.1),
                    new BallDetection(-0.1, 2.2, 2.2),
                    new BallDetection(0.0, 2.3, 2.3)
                  };
              inputs.camera1Detections = new BallDetection[0];
              return null;
            })
        .when(io)
        .updateInputs(any());

    subsystem.periodic();
    assertTrue(subsystem.getBestPath().isPresent());

    subsystem.setCommitted(true);
    subsystem.setCommitted(false);

    subsystem.periodic();
    assertTrue(subsystem.getBestPath().isPresent(), "Should replan after recommitting to false");
  }
}
