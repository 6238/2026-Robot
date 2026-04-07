package subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

class VisionTest {
  static final double DELTA = 1e-9;

  // A safe in-bounds point on the 2026 field (~17.55 x 8.21 m)
  static final double FIELD_X = 8.0;
  static final double FIELD_Y = 4.0;

  VisionIO mockIO;
  VisionConsumer mockConsumer;
  Vision vision;

  @BeforeAll
  static void initHAL() {
    // Alert construction + Alert.set() require HAL/NT to be initialized
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setup() {
    mockIO = Mockito.mock(VisionIO.class);
    mockConsumer = Mockito.mock(VisionConsumer.class);
    // Default: camera connected with no observations so Alert never fires
    defaultConnectedInputs(mockIO);
    vision = new Vision(mockConsumer, Pose2d::new, mockIO);
  }

  // ── helpers ───────────────────────────────────────────────────────────────

  private void defaultConnectedInputs(VisionIO io) {
    doAnswer(
            inv -> {
              VisionIOInputs inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.poseObservations = new PoseObservation[0];
              inputs.tagIds = new int[0];
              return null;
            })
        .when(io)
        .updateInputs(any(VisionIOInputs.class), any(Pose2d.class));
  }

  private void setObservations(VisionIO io, PoseObservation... obs) {
    doAnswer(
            inv -> {
              VisionIOInputs inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.poseObservations = obs;
              inputs.tagIds = new int[0];
              return null;
            })
        .when(io)
        .updateInputs(any(VisionIOInputs.class), any(Pose2d.class));
  }

  /** A valid in-bounds, single-tag, zero-ambiguity observation at the given distance. */
  private PoseObservation validObservation(double distance) {
    return new PoseObservation(
        1.0,
        new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
        0.0,
        1,
        distance,
        PoseObservationType.MEGATAG_2);
  }

  // ── pose rejection: zero tags ─────────────────────────────────────────────

  @Test
  void zeroTags_rejectsObservation() {
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
            0.0,
            0, // zero tags
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verifyNoInteractions(mockConsumer);
  }

  // ── pose rejection: single-tag ambiguity ──────────────────────────────────

  // @Test
  // void singleTag_highAmbiguity_rejectsObservation() {
  //   setObservations(
  //       mockIO,
  //       new PoseObservation(
  //           1.0,
  //           new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
  //           0.08, // > maxAmbiguity (0.07)
  //           1,
  //           2.0,
  //           PoseObservationType.MEGATAG_2));
  //   vision.periodic();
  //   verifyNoInteractions(mockConsumer);
  // }

  @Test
  void singleTag_ambiguityAtThreshold_acceptsObservation() {
    // Rejection condition is ambiguity > maxAmbiguity (strictly greater), so 0.07 is accepted
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
            0.07, // exactly at threshold — not rejected
            1,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verify(mockConsumer, times(1)).accept(any(), anyDouble(), any());
  }

  @Test
  void singleTag_lowAmbiguity_acceptsObservation() {
    setObservations(mockIO, validObservation(2.0));
    vision.periodic();
    verify(mockConsumer, times(1)).accept(any(), anyDouble(), any());
  }

  // ── pose rejection: Z error ───────────────────────────────────────────────

  @Test
  void badPositiveZ_rejectsObservation() {
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, 0.8, new Rotation3d()), // |Z| > 0.75
            0.0,
            1,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verifyNoInteractions(mockConsumer);
  }

  @Test
  void badNegativeZ_rejectsObservation() {
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, -0.8, new Rotation3d()), // |Z| > 0.75
            0.0,
            1,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verifyNoInteractions(mockConsumer);
  }

  // ── pose rejection: out-of-field bounds ───────────────────────────────────

  @Test
  void negativeX_rejectsObservation() {
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(-0.1, FIELD_Y, 0.0, new Rotation3d()),
            0.0,
            1,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verifyNoInteractions(mockConsumer);
  }

  @Test
  void negativeY_rejectsObservation() {
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, -0.1, 0.0, new Rotation3d()),
            0.0,
            1,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verifyNoInteractions(mockConsumer);
  }

  // ── multi-tag: ambiguity check is skipped ────────────────────────────────

  @Test
  void multiTag_highAmbiguity_acceptsObservation() {
    // Ambiguity rejection only applies when tagCount == 1
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
            0.9, // very high ambiguity — irrelevant for multi-tag
            2,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verify(mockConsumer, times(1)).accept(any(), anyDouble(), any());
  }

  // ── standard deviation calculation ────────────────────────────────────────

  @Test
  @SuppressWarnings("unchecked")
  void stdDev_singleTag_distance1() {
    // stdDevFactor = 1^2 / 1 = 1.0  →  linear = 0.07 * 1.0 = 0.07, angular = 0.09 * 1.0 = 0.09
    setObservations(mockIO, validObservation(1.0));
    vision.periodic();

    ArgumentCaptor<Matrix<N3, N1>> captor = ArgumentCaptor.forClass(Matrix.class);
    verify(mockConsumer).accept(any(), anyDouble(), captor.capture());

    Matrix<N3, N1> stdDevs = captor.getValue();
    assertEquals(0.07, stdDevs.get(0, 0), DELTA);
    assertEquals(0.07, stdDevs.get(1, 0), DELTA);
    assertEquals(0.09, stdDevs.get(2, 0), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void stdDev_singleTag_distance2() {
    // stdDevFactor = 2^2 / 1 = 4.0  →  linear = 0.07 * 4.0 = 0.28, angular = 0.09 * 4.0 = 0.36
    setObservations(mockIO, validObservation(2.0));
    vision.periodic();

    ArgumentCaptor<Matrix<N3, N1>> captor = ArgumentCaptor.forClass(Matrix.class);
    verify(mockConsumer).accept(any(), anyDouble(), captor.capture());

    Matrix<N3, N1> stdDevs = captor.getValue();
    assertEquals(0.28, stdDevs.get(0, 0), DELTA);
    assertEquals(0.28, stdDevs.get(1, 0), DELTA);
    assertEquals(0.36, stdDevs.get(2, 0), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void stdDev_twoTags_distance2() {
    // stdDevFactor = 2^2 / 2 = 2.0  →  linear = 0.07 * 2.0 = 0.14, angular = 0.09 * 2.0 = 0.18
    setObservations(
        mockIO,
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
            0.0,
            2,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();

    ArgumentCaptor<Matrix<N3, N1>> captor = ArgumentCaptor.forClass(Matrix.class);
    verify(mockConsumer).accept(any(), anyDouble(), captor.capture());

    Matrix<N3, N1> stdDevs = captor.getValue();
    assertEquals(0.14, stdDevs.get(0, 0), DELTA);
    assertEquals(0.14, stdDevs.get(1, 0), DELTA);
    assertEquals(0.18, stdDevs.get(2, 0), DELTA);
  }

  // ── consumer receives correct pose and timestamp ──────────────────────────

  @Test
  void consumer_receivesCorrectPose() {
    setObservations(mockIO, validObservation(1.0));
    vision.periodic();

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockConsumer).accept(poseCaptor.capture(), anyDouble(), any());

    Pose2d captured = poseCaptor.getValue();
    assertEquals(FIELD_X, captured.getX(), DELTA);
    assertEquals(FIELD_Y, captured.getY(), DELTA);
  }

  @Test
  void consumer_receivesCorrectTimestamp() {
    double expectedTimestamp = 3.14;
    setObservations(
        mockIO,
        new PoseObservation(
            expectedTimestamp,
            new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
            0.0,
            1,
            1.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();

    ArgumentCaptor<Double> tsCaptor = ArgumentCaptor.forClass(Double.class);
    verify(mockConsumer).accept(any(), tsCaptor.capture(), any());
    assertEquals(expectedTimestamp, tsCaptor.getValue(), DELTA);
  }

  // ── getTargetX ────────────────────────────────────────────────────────────

  @Test
  void getTargetX_returnsLatestTargetObservationTx() {
    Rotation2d expectedTx = Rotation2d.fromDegrees(15.0);
    doAnswer(
            inv -> {
              VisionIOInputs inputs = inv.getArgument(0);
              inputs.connected = true;
              inputs.poseObservations = new PoseObservation[0];
              inputs.tagIds = new int[0];
              inputs.latestTargetObservation = new TargetObservation(expectedTx, new Rotation2d());
              return null;
            })
        .when(mockIO)
        .updateInputs(any(VisionIOInputs.class), any(Pose2d.class));

    vision.periodic();

    assertEquals(expectedTx.getRadians(), vision.getTargetX(0).getRadians(), DELTA);
  }

  @Test
  void getTargetX_defaultsToZeroBeforePeriodic() {
    // Before periodic() is called, latestTargetObservation is initialized to Rotation2d() = 0
    assertEquals(0.0, vision.getTargetX(0).getRadians(), DELTA);
  }

  // ── multiple observations in one frame ───────────────────────────────────

  @Test
  void multipleObservations_callsConsumerForEachAccepted() {
    setObservations(mockIO, validObservation(1.0), validObservation(2.0));
    vision.periodic();
    verify(mockConsumer, times(2)).accept(any(), anyDouble(), any());
  }

  @Test
  void mixedObservations_onlyAcceptedReachConsumer() {
    // First: valid; second: zero tags (rejected)
    setObservations(
        mockIO,
        validObservation(1.0),
        new PoseObservation(
            1.0,
            new Pose3d(FIELD_X, FIELD_Y, 0.0, new Rotation3d()),
            0.0,
            0,
            2.0,
            PoseObservationType.MEGATAG_2));
    vision.periodic();
    verify(mockConsumer, times(1)).accept(any(), anyDouble(), any());
  }

  // ── multi-camera ──────────────────────────────────────────────────────────

  @Test
  void multiCamera_eachCameraCallsUpdateInputs() {
    VisionIO mockIO2 = Mockito.mock(VisionIO.class);
    defaultConnectedInputs(mockIO2);
    Vision multiVision = new Vision(mockConsumer, Pose2d::new, mockIO, mockIO2);

    multiVision.periodic();

    verify(mockIO).updateInputs(any(VisionIOInputs.class), any(Pose2d.class));
    verify(mockIO2).updateInputs(any(VisionIOInputs.class), any(Pose2d.class));
  }

  @Test
  void multiCamera_bothCamerasContributeObservations() {
    VisionIO mockIO2 = Mockito.mock(VisionIO.class);
    setObservations(mockIO, validObservation(1.0));
    setObservations(mockIO2, validObservation(1.0));
    Vision multiVision = new Vision(mockConsumer, Pose2d::new, mockIO, mockIO2);

    multiVision.periodic();

    // One accepted observation from each camera
    verify(mockConsumer, times(2)).accept(any(), anyDouble(), any());
  }

  // ── periodic: updateInputs called ────────────────────────────────────────

  @Test
  void periodic_callsUpdateInputs() {
    vision.periodic();
    verify(mockIO).updateInputs(any(VisionIOInputs.class), any(Pose2d.class));
  }

  @Test
  void periodic_passesCurrentPoseEstimateToIO() {
    Pose2d suppliedPose = new Pose2d(3.0, 4.0, new Rotation2d());
    Vision visionWithPose = new Vision(mockConsumer, () -> suppliedPose, mockIO);

    visionWithPose.periodic();

    ArgumentCaptor<Pose2d> poseCaptor = ArgumentCaptor.forClass(Pose2d.class);
    verify(mockIO).updateInputs(any(VisionIOInputs.class), poseCaptor.capture());
    assertEquals(suppliedPose.getX(), poseCaptor.getValue().getX(), DELTA);
    assertEquals(suppliedPose.getY(), poseCaptor.getValue().getY(), DELTA);
  }
}
