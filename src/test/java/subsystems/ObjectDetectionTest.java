// package subsystems;

// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.junit.jupiter.api.Assertions.assertTrue;
// import static org.mockito.ArgumentMatchers.any;
// import static org.mockito.ArgumentMatchers.anyDouble;
// import static org.mockito.Mockito.doAnswer;
// import static org.mockito.Mockito.when;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.simulation.SimHooks;
// import frc.robot.subsystems.objectdetection.ObjectDetection;
// import frc.robot.subsystems.objectdetection.ObjectDetection.TrackedObject;
// import frc.robot.subsystems.objectdetection.ObjectDetectionConstants;
// import frc.robot.subsystems.objectdetection.ObjectDetectionIO;
// import frc.robot.subsystems.objectdetection.ObjectDetectionIO.TargetObservation;
// import frc.robot.subsystems.objectdetection.ObjectDetectionIOInputsAutoLogged;
// import java.util.Optional;
// import java.util.function.DoubleFunction;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.mockito.Mock;
// import org.mockito.MockitoAnnotations;

// public class ObjectDetectionTest {

//   @Mock private ObjectDetectionIO io;
//   @Mock private DoubleFunction<Optional<Pose2d>> timestampPoseFunction;

//   private ObjectDetection objectDetection;

//   @BeforeEach
//   public void setup() {
//     MockitoAnnotations.openMocks(this);

//     assert HAL.initialize(500, 0);
//     SimHooks.restartTiming();
//     objectDetection = new ObjectDetection(io, timestampPoseFunction);
//   }

//   @AfterEach
//   public void teardown() {
//     SimHooks.pauseTiming();
//   }

//   @Test
//   public void testInitialization() {
//     assertEquals(0, objectDetection.getTrackedObjects().length);
//   }

//   @Test
//   public void testPeriodicAddsNewObject() {
//     double timestamp = 1.0;
//     Pose2d robotPose = new Pose2d(10.0, 10.0, new Rotation2d());

//     when(timestampPoseFunction.apply(timestamp)).thenReturn(Optional.of(robotPose));

//     // Setup inputs: Object 2m in front of robot
//     TargetObservation obs = new TargetObservation(2.0, 0.0, 0.0, 0.0, timestamp);

//     doAnswer(
//             invocation -> {
//               ObjectDetectionIOInputsAutoLogged inputs = invocation.getArgument(0);
//               inputs.targetObservations = new TargetObservation[] {obs};
//               return null;
//             })
//         .when(io)
//         .updateInputs(any());

//     // Execute
//     objectDetection.periodic();

//     // Verify
//     TrackedObject[] objects = objectDetection.getTrackedObjects();
//     assertEquals(1, objects.length);

//     // Calculate expected pose using constants to be robust
//     Pose2d expectedPose =
//         robotPose
//             .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
//             .transformBy(new Transform2d(new Translation2d(2.0, 0.0), new Rotation2d()));

//     Translation2d trackedTranslation = objects[0].getPose().getTranslation();
//     assertEquals(expectedPose.getX(), trackedTranslation.getX(), 0.01);
//     assertEquals(expectedPose.getY(), trackedTranslation.getY(), 0.01);
//   }

//   @Test
//   public void testPeriodicUpdatesExistingObject() {
//     double t1 = 1.0;
//     double t2 = 1.1;
//     Pose2d robotPose = new Pose2d();
//     when(timestampPoseFunction.apply(anyDouble())).thenReturn(Optional.of(robotPose));

//     // Step 1: Add object at (2,0)
//     TargetObservation obs1 = new TargetObservation(2.0, 0.0, 0.0, 0.0, t1);
//     doAnswer(
//             invocation -> {
//               ObjectDetectionIOInputsAutoLogged inputs = invocation.getArgument(0);
//               inputs.targetObservations = new TargetObservation[] {obs1};
//               return null;
//             })
//         .when(io)
//         .updateInputs(any());

//     objectDetection.periodic();
//     double initialX = objectDetection.getTrackedObjects()[0].getPose().getX();

//     // Step 2: Update object observed at (3,0)
//     TargetObservation obs2 = new TargetObservation(2.5, 0.0, 0.0, 0.0, t2);
//     doAnswer(
//             invocation -> {
//               ObjectDetectionIOInputsAutoLogged inputs = invocation.getArgument(0);
//               inputs.targetObservations = new TargetObservation[] {obs2};
//               return null;
//             })
//         .when(io)
//         .updateInputs(any());

//     objectDetection.periodic();

//     // Verify
//     assertEquals(1, objectDetection.getTrackedObjects().length);
//     double updatedX = objectDetection.getTrackedObjects()[0].getPose().getX();

//     // Filter should move value towards 3.0, so it should be greater than initial 2.0
//     assertTrue(updatedX > initialX);
//     // Should update timestamp
//     assertEquals(t2, objectDetection.getTrackedObjects()[0].getLastSeenTimestamp(), 0.001);
//   }

//   @Test
//   public void testPeriodicRemovesTimedOutObjects() {
//     double startTime = 0.0;
//     Pose2d robotPose = new Pose2d();
//     when(timestampPoseFunction.apply(anyDouble())).thenReturn(Optional.of(robotPose));

//     // Add object
//     TargetObservation obs = new TargetObservation(2.0, 0.0, 0.0, 0.0, startTime);
//     doAnswer(
//             invocation -> {
//               ObjectDetectionIOInputsAutoLogged inputs = invocation.getArgument(0);
//               inputs.targetObservations = new TargetObservation[] {obs};
//               return null;
//             })
//         .when(io)
//         .updateInputs(any());

//     objectDetection.periodic();
//     assertEquals(1, objectDetection.getTrackedObjects().length);

//     // Advance time past timeout threshold
//     double timeoutDuration = ObjectDetectionConstants.OBJECT_HISTORY_TIMEOUT.in(Units.Seconds);
//     SimHooks.stepTiming(timeoutDuration + 0.1);

//     // Update with no observations
//     doAnswer(
//             invocation -> {
//               ObjectDetectionIOInputsAutoLogged inputs = invocation.getArgument(0);
//               inputs.targetObservations = new TargetObservation[] {};
//               return null;
//             })
//         .when(io)
//         .updateInputs(any());

//     objectDetection.periodic();

//     // Verify removal
//     assertEquals(0, objectDetection.getTrackedObjects().length);
//   }

//   @Test
//   public void testClosestTrackedObject() {
//     // Manually seed tracked objects
//     // Create mocks or setup state via periodic
//     Pose2d robotPose = new Pose2d();
//     when(timestampPoseFunction.apply(anyDouble())).thenReturn(Optional.of(robotPose));

//     // Obs A: 2m away
//     TargetObservation obsA = new TargetObservation(2.0, 0.0, 0.0, 0.0, 0.0);
//     // Obs B: 10m away
//     TargetObservation obsB = new TargetObservation(10.0, 0.0, 0.0, 0.0, 0.0);

//     doAnswer(
//             invocation -> {
//               ObjectDetectionIOInputsAutoLogged inputs = invocation.getArgument(0);
//               inputs.targetObservations = new TargetObservation[] {obsA, obsB};
//               return null;
//             })
//         .when(io)
//         .updateInputs(any());

//     objectDetection.periodic();

//     TrackedObject[] objects = objectDetection.getTrackedObjects();
//     assertEquals(2, objects.length);

//     // Test Query 1: Close to A
//     Pose2d queryPoseA =
//         robotPose
//             .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
//             .transformBy(new Transform2d(new Translation2d(2.0, 0.0), new Rotation2d()));

//     Optional<TrackedObject> resultA = objectDetection.closestTrackedObject(queryPoseA);
//     assertTrue(resultA.isPresent());
//     // Verify we got the object near 2.0, not 10.0
//     // (Assuming simple robot-frame alignment for this test case)
//     double distA =
//         resultA.get().getPose().getTranslation().getDistance(queryPoseA.getTranslation());
//     assertTrue(distA < 2.0);

//     // Test Query 2: Close to B
//     Pose2d queryPoseB =
//         robotPose
//             .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
//             .transformBy(new Transform2d(new Translation2d(10.0, 0.0), new Rotation2d()));

//     Optional<TrackedObject> resultB = objectDetection.closestTrackedObject(queryPoseB);
//     assertTrue(resultB.isPresent());
//     double distB =
//         resultB.get().getPose().getTranslation().getDistance(queryPoseB.getTranslation());
//     assertTrue(distB < 2.0);
//   }

//   @Test
//   public void testClosestTrackedObjectEmpty() {
//     assertEquals(Optional.empty(), objectDetection.closestTrackedObject(new Pose2d()));
//     assertEquals(Optional.empty(), objectDetection.closestTrackedObjectPose(new Pose2d()));
//   }

//   @Test
//   public void testGetObjectWorldPose() {
//     double timestamp = 1.5;
//     Pose2d robotPose = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(90));
//     when(timestampPoseFunction.apply(timestamp)).thenReturn(Optional.of(robotPose));

//     TargetObservation obs = new TargetObservation(0, 2.0, 1.0, 0.0, timestamp);

//     Optional<Pose2d> result = objectDetection.getObjectWorldPose(obs);

//     assertTrue(result.isPresent());

//     // Expected Math: Robot -> Camera -> Observation
//     Pose2d expected =
//         robotPose
//             .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)
//             .transformBy(new Transform2d(new Translation2d(obs.dx(), obs.dy()), new
// Rotation2d()));

//     assertEquals(expected.getX(), result.get().getX(), 0.001);
//     assertEquals(expected.getY(), result.get().getY(), 0.001);
//   }

//   @Test
//   public void testGetObjectWorldPoseMissingRobotPose() {
//     when(timestampPoseFunction.apply(anyDouble())).thenReturn(Optional.empty());
//     TargetObservation obs = new TargetObservation(0, 1.0, 0.0, 0.0, 0.0);

//     Optional<Pose2d> result = objectDetection.getObjectWorldPose(obs);
//     assertTrue(result.isEmpty());
//   }
// }
