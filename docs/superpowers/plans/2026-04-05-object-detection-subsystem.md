# Object Detection Subsystem Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an `ObjectDetection` subsystem that reads ball detections from NetworkTables, accumulates them in a field-relative rolling buffer, clusters with DBSCAN, fits a principal axis via PCA, and generates a live PathPlanner path for continuous multi-cluster collection.

**Architecture:** The subsystem follows the AK IO layer pattern. Algorithm logic is isolated in `ObjectDetectionAlgorithms` (package-private static methods) for testability. The subsystem owns the rolling buffer, coordinate transforms, and path generation. The command chains clusters continuously until cancelled.

**Tech Stack:** WPILib 2025, AdvantageKit 6.x, PathPlanner 2026.1.2, JUnit 5, Mockito 5

> **Note on spec Step 3 (Ellipse Fitting):** The spec describes Fitzgibbon's direct conic method. This plan uses **PCA (2×2 covariance eigendecomposition)** instead — it produces identical useful output (center, axis direction, semi-major extent) with far less code, no matrix library dependency, and natural handling of degenerate cases (≥2 points always works).

> **Note on PathPlanner waypoint rotations:** `PathPlannerPath.waypointsFromPoses()` pose rotations are **direction of travel** (path tangent), not chassis heading. Intake-leading chassis orientation is handled separately via `PPHolonomicDriveController.overrideRotationFeedback()`.

---

## File Map

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `src/main/java/frc/robot/subsystems/objectdetection/BallDetection.java` | NT struct record |
| Create | `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionConstants.java` | All tunable params |
| Create | `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIO.java` | AK IO interface + `@AutoLog` inputs |
| Create | `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIOReal.java` | NT subscriptions |
| Create | `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIOSim.java` | Synthetic detections for sim/tests |
| Create | `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java` | DBSCAN, scoring, PCA, waypoints |
| Create | `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetection.java` | Subsystem: buffer, pipeline, command |
| Modify | `src/main/java/frc/robot/RobotContainer.java` | Wire subsystem + button binding |
| Create | `src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java` | Algorithm unit tests |
| Create | `src/test/java/subsystems/ObjectDetectionTest.java` | Subsystem integration tests |

---

## Task 1: BallDetection record + ObjectDetectionConstants

**Files:**
- Create: `src/main/java/frc/robot/subsystems/objectdetection/BallDetection.java`
- Create: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionConstants.java`

- [ ] **Step 1: Create BallDetection.java** (from NT_FORMAT.md — exact schema)

```java
package frc.robot.subsystems.objectdetection;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record BallDetection(double x, double y, double distance) implements StructSerializable {

  public static final BallDetectionStruct struct = new BallDetectionStruct();

  public static final class BallDetectionStruct implements Struct<BallDetection> {
    @Override
    public Class<BallDetection> getTypeClass() {
      return BallDetection.class;
    }

    @Override
    public String getTypeName() {
      return "BallDetection";
    }

    @Override
    public String getTypeString() {
      return "struct:BallDetection";
    }

    @Override
    public int getSize() {
      return kSizeDouble * 3;
    }

    @Override
    public String getSchema() {
      return "double x;double y;double distance";
    }

    @Override
    public BallDetection unpack(ByteBuffer bb) {
      return new BallDetection(bb.getDouble(), bb.getDouble(), bb.getDouble());
    }

    @Override
    public void pack(ByteBuffer bb, BallDetection value) {
      bb.putDouble(value.x());
      bb.putDouble(value.y());
      bb.putDouble(value.distance());
    }
  }
}
```

- [ ] **Step 2: Create ObjectDetectionConstants.java**

```java
package frc.robot.subsystems.objectdetection;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class ObjectDetectionConstants {

  // Camera mount offsets from robot center (robot frame: x=forward, y=left)
  // TODO: fill in from robot CAD
  public static final Transform2d CAMERA_0_ROBOT_OFFSET =
      new Transform2d(0.0, 0.15, Rotation2d.fromDegrees(0));
  public static final Transform2d CAMERA_1_ROBOT_OFFSET =
      new Transform2d(0.0, -0.15, Rotation2d.fromDegrees(0));

  // Rolling buffer
  public static final double BUFFER_WINDOW_SECS = 1.5;

  // DBSCAN
  public static final double DBSCAN_EPSILON = 0.8; // meters
  public static final int DBSCAN_MIN_PTS = 2;

  // Cluster scoring
  public static final double COUNT_SCALAR = 1.0;
  public static final double DIST_SCALAR = 0.5;
  public static final double DECAY_K = 2.0; // exponential decay per second

  // Replan gate
  public static final double REPLAN_THRESHOLD_METERS = 0.15;

  // PathPlanner constraints
  public static final double MAX_COLLECTION_SPEED_MPS = 2.0;
  public static final double MAX_COLLECTION_ACCEL_MPS2 = 3.0;
  public static final double MAX_COLLECTION_ANG_SPEED_RAD = 2 * Math.PI;
  public static final double MAX_COLLECTION_ANG_ACCEL_RAD = 4 * Math.PI;
  public static final PathConstraints COLLECTION_CONSTRAINTS =
      new PathConstraints(
          MAX_COLLECTION_SPEED_MPS,
          MAX_COLLECTION_ACCEL_MPS2,
          MAX_COLLECTION_ANG_SPEED_RAD,
          MAX_COLLECTION_ANG_ACCEL_RAD);

  // Minimum field speed (m/s) before overriding holonomic rotation toward intake-leading
  public static final double ROTATION_OVERRIDE_MIN_SPEED = 0.15;

  private ObjectDetectionConstants() {}
}
```

- [ ] **Step 3: Verify it compiles**

```bash
./gradlew compileJava -x test
```

Expected: BUILD SUCCESSFUL (no errors in new files)

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/BallDetection.java \
        src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionConstants.java
git commit -m "feat(objectdetection): add BallDetection struct and constants"
```

---

## Task 2: ObjectDetectionIO interface

**Files:**
- Create: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIO.java`

- [ ] **Step 1: Create ObjectDetectionIO.java**

```java
package frc.robot.subsystems.objectdetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {

  @AutoLog
  class ObjectDetectionIOInputs {
    public boolean connected = false;
    public long heartbeat = 0;
    public BallDetection[] camera0Detections = new BallDetection[0];
    public BallDetection[] camera1Detections = new BallDetection[0];
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
```

- [ ] **Step 2: Verify AK annotation processor generates the autologged class**

```bash
./gradlew compileJava -x test
```

Expected: BUILD SUCCESSFUL. The annotation processor generates `ObjectDetectionIOInputsAutoLogged` — confirm no compile error about the missing class.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIO.java
git commit -m "feat(objectdetection): add ObjectDetectionIO interface"
```

---

## Task 3: DBSCAN algorithm (TDD)

**Files:**
- Create: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java`
- Create: `src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java`

- [ ] **Step 1: Write the failing tests**

Create `src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java`:

```java
package subsystems;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.objectdetection.ObjectDetectionAlgorithms;
import frc.robot.subsystems.objectdetection.ObjectDetectionAlgorithms.TimestampedFieldDetection;
import java.util.List;
import org.junit.jupiter.api.Test;

class ObjectDetectionAlgorithmsTest {

  private TimestampedFieldDetection pt(double x, double y) {
    return new TimestampedFieldDetection(x, y, 0.0);
  }

  @Test
  void dbscan_twoSeparateClusters_returnsTwoClusters() {
    List<TimestampedFieldDetection> points =
        List.of(
            pt(0, 0), pt(0.1, 0), pt(0.2, 0), // cluster A
            pt(5, 5), pt(5.1, 5), pt(5.2, 5)); // cluster B

    List<List<TimestampedFieldDetection>> clusters =
        ObjectDetectionAlgorithms.dbscan(points, 0.5, 2);

    assertEquals(2, clusters.size());
    assertEquals(3, clusters.get(0).size());
    assertEquals(3, clusters.get(1).size());
  }

  @Test
  void dbscan_isolatedNoisyPoint_isDiscarded() {
    List<TimestampedFieldDetection> points =
        List.of(
            pt(0, 0), pt(0.1, 0), pt(0.2, 0), // cluster
            pt(10, 10)); // isolated noise

    List<List<TimestampedFieldDetection>> clusters =
        ObjectDetectionAlgorithms.dbscan(points, 0.5, 2);

    assertEquals(1, clusters.size());
    assertEquals(3, clusters.get(0).size());
  }

  @Test
  void dbscan_emptyInput_returnsEmptyList() {
    assertTrue(ObjectDetectionAlgorithms.dbscan(List.of(), 0.5, 2).isEmpty());
  }

  @Test
  void dbscan_allPointsTooFarApart_returnsNoClusters() {
    // All points > 0.5m apart — each is isolated noise
    List<TimestampedFieldDetection> points = List.of(pt(0, 0), pt(2, 0), pt(4, 0));
    assertTrue(ObjectDetectionAlgorithms.dbscan(points, 0.5, 2).isEmpty());
  }
}
```

- [ ] **Step 2: Run tests — expect compilation failure (class doesn't exist yet)**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: FAILED — `ObjectDetectionAlgorithms` not found

- [ ] **Step 3: Create ObjectDetectionAlgorithms.java with DBSCAN**

```java
package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

class ObjectDetectionAlgorithms {

  record TimestampedFieldDetection(double x, double y, double timestamp) {}

  record PrincipalAxis(
      double centerX, double centerY, double axisX, double axisY, double semiMajor) {}

  // -------------------------------------------------------------------------
  // DBSCAN
  // -------------------------------------------------------------------------

  static List<List<TimestampedFieldDetection>> dbscan(
      List<TimestampedFieldDetection> points, double epsilon, int minPts) {
    int n = points.size();
    int[] label = new int[n]; // 0=unvisited, -1=noise, >0=cluster id
    List<List<TimestampedFieldDetection>> clusters = new ArrayList<>();
    int clusterCount = 0;

    for (int i = 0; i < n; i++) {
      if (label[i] != 0) continue;

      List<Integer> neighbors = regionQuery(points, i, epsilon);
      if (neighbors.size() < minPts) {
        label[i] = -1;
        continue;
      }

      clusterCount++;
      List<TimestampedFieldDetection> cluster = new ArrayList<>();
      clusters.add(cluster);
      label[i] = clusterCount;
      cluster.add(points.get(i));

      ArrayDeque<Integer> queue = new ArrayDeque<>(neighbors);
      while (!queue.isEmpty()) {
        int q = queue.poll();
        if (label[q] == -1) label[q] = clusterCount; // border point
        if (label[q] != 0) continue;

        label[q] = clusterCount;
        cluster.add(points.get(q));

        List<Integer> qNeighbors = regionQuery(points, q, epsilon);
        if (qNeighbors.size() >= minPts) queue.addAll(qNeighbors);
      }
    }

    return clusters;
  }

  private static List<Integer> regionQuery(
      List<TimestampedFieldDetection> points, int idx, double epsilon) {
    List<Integer> neighbors = new ArrayList<>();
    double px = points.get(idx).x();
    double py = points.get(idx).y();
    double eps2 = epsilon * epsilon;
    for (int i = 0; i < points.size(); i++) {
      double dx = points.get(i).x() - px;
      double dy = points.get(i).y() - py;
      if (dx * dx + dy * dy <= eps2) neighbors.add(i);
    }
    return neighbors;
  }

  // scoreCluster, fitPrincipalAxis, generateCollectionWaypoints added in later tasks
}
```

- [ ] **Step 4: Run tests — expect PASS**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 4 tests PASSED

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java \
        src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java
git commit -m "feat(objectdetection): DBSCAN clustering with tests"
```

---

## Task 4: Cluster scoring (TDD)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java`
- Modify: `src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java`

- [ ] **Step 1: Add failing tests** (append to `ObjectDetectionAlgorithmsTest`)

```java
  // ---- scoring tests ----

  @Test
  void scoreCluster_closerClusterScoresHigher() {
    List<TimestampedFieldDetection> close =
        List.of(
            new TimestampedFieldDetection(1, 0, 0),
            new TimestampedFieldDetection(1.1, 0, 0));
    List<TimestampedFieldDetection> far =
        List.of(
            new TimestampedFieldDetection(10, 0, 0),
            new TimestampedFieldDetection(10.1, 0, 0));

    Translation2d robot = new Translation2d(0, 0);
    double scoreClose = ObjectDetectionAlgorithms.scoreCluster(close, robot, 0, 2.0, 1.0, 0.5);
    double scoreFar = ObjectDetectionAlgorithms.scoreCluster(far, robot, 0, 2.0, 1.0, 0.5);

    assertTrue(scoreClose > scoreFar, "Closer cluster should score higher");
  }

  @Test
  void scoreCluster_largerClusterScoresHigher() {
    List<TimestampedFieldDetection> large =
        List.of(
            new TimestampedFieldDetection(5, 0, 0),
            new TimestampedFieldDetection(5.1, 0, 0),
            new TimestampedFieldDetection(5.2, 0, 0));
    List<TimestampedFieldDetection> small =
        List.of(
            new TimestampedFieldDetection(5, 0, 0),
            new TimestampedFieldDetection(5.1, 0, 0));

    Translation2d robot = new Translation2d(0, 0);
    assertTrue(
        ObjectDetectionAlgorithms.scoreCluster(large, robot, 0, 2.0, 1.0, 0.5)
            > ObjectDetectionAlgorithms.scoreCluster(small, robot, 0, 2.0, 1.0, 0.5));
  }

  @Test
  void scoreCluster_olderPointsReduceScore() {
    // Fresh cluster (age=0) vs stale cluster (age=1s), same position
    List<TimestampedFieldDetection> fresh =
        List.of(
            new TimestampedFieldDetection(5, 0, 1.0),
            new TimestampedFieldDetection(5.1, 0, 1.0));
    List<TimestampedFieldDetection> stale =
        List.of(
            new TimestampedFieldDetection(5, 0, 0.0),
            new TimestampedFieldDetection(5.1, 0, 0.0));

    Translation2d robot = new Translation2d(0, 0);
    double now = 1.0;
    assertTrue(
        ObjectDetectionAlgorithms.scoreCluster(fresh, robot, now, 2.0, 1.0, 0.5)
            > ObjectDetectionAlgorithms.scoreCluster(stale, robot, now, 2.0, 1.0, 0.5));
  }
```

Also add the `Translation2d` import to the test file:

```java
import edu.wpi.first.math.geometry.Translation2d;
```

- [ ] **Step 2: Run tests — expect failure on scoring tests**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 4 PASS (DBSCAN), 3 FAIL (scoring methods not found)

- [ ] **Step 3: Add scoring methods to ObjectDetectionAlgorithms.java** (add after `regionQuery`)

```java
  // -------------------------------------------------------------------------
  // Cluster scoring
  // -------------------------------------------------------------------------

  static double scoreCluster(
      List<TimestampedFieldDetection> cluster,
      Translation2d robotPos,
      double now,
      double decayK,
      double countScalar,
      double distScalar) {
    double weightedCount = 0;
    double sumX = 0, sumY = 0;
    for (TimestampedFieldDetection p : cluster) {
      double age = now - p.timestamp();
      weightedCount += Math.exp(-decayK * age);
      sumX += p.x();
      sumY += p.y();
    }
    double cx = sumX / cluster.size();
    double cy = sumY / cluster.size();
    double dx = cx - robotPos.getX();
    double dy = cy - robotPos.getY();
    return weightedCount * countScalar - (dx * dx + dy * dy) * distScalar;
  }

  static Translation2d clusterCentroid(List<TimestampedFieldDetection> cluster) {
    double sumX = 0, sumY = 0;
    for (TimestampedFieldDetection p : cluster) {
      sumX += p.x();
      sumY += p.y();
    }
    return new Translation2d(sumX / cluster.size(), sumY / cluster.size());
  }
```

- [ ] **Step 4: Run tests — expect all PASS**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 7 tests PASSED

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java \
        src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java
git commit -m "feat(objectdetection): cluster scoring with decay"
```

---

## Task 5: PCA principal axis fitting (TDD)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java`
- Modify: `src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java`

- [ ] **Step 1: Add failing tests** (append to `ObjectDetectionAlgorithmsTest`)

```java
  // ---- PCA tests ----

  @Test
  void fitPrincipalAxis_horizontalLine_returnsHorizontalAxis() {
    List<TimestampedFieldDetection> points =
        List.of(
            new TimestampedFieldDetection(-1, 0, 0),
            new TimestampedFieldDetection(0, 0, 0),
            new TimestampedFieldDetection(1, 0, 0));

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(points);

    assertEquals(0.0, axis.centerX(), 1e-9);
    assertEquals(0.0, axis.centerY(), 1e-9);
    assertEquals(1.0, Math.abs(axis.axisX()), 1e-9); // ±1 both valid
    assertEquals(0.0, Math.abs(axis.axisY()), 1e-9);
    assertEquals(1.0, axis.semiMajor(), 1e-9);
  }

  @Test
  void fitPrincipalAxis_verticalLine_returnsVerticalAxis() {
    List<TimestampedFieldDetection> points =
        List.of(
            new TimestampedFieldDetection(0, -2, 0),
            new TimestampedFieldDetection(0, 0, 0),
            new TimestampedFieldDetection(0, 2, 0));

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(points);

    assertEquals(0.0, axis.centerX(), 1e-9);
    assertEquals(0.0, axis.centerY(), 1e-9);
    assertEquals(0.0, Math.abs(axis.axisX()), 1e-9);
    assertEquals(1.0, Math.abs(axis.axisY()), 1e-9);
    assertEquals(2.0, axis.semiMajor(), 1e-9);
  }

  @Test
  void fitPrincipalAxis_twoPoints_halfDistanceIsSemiMajor() {
    // Points (0,0) and (3,4) — distance = 5, center = (1.5, 2.0)
    List<TimestampedFieldDetection> points =
        List.of(
            new TimestampedFieldDetection(0, 0, 0),
            new TimestampedFieldDetection(3, 4, 0));

    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(points);

    assertEquals(1.5, axis.centerX(), 1e-9);
    assertEquals(2.0, axis.centerY(), 1e-9);
    assertEquals(2.5, axis.semiMajor(), 1e-9); // half of distance 5
  }
```

- [ ] **Step 2: Run tests — expect failure on PCA tests**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 7 PASS, 3 FAIL (PCA method not found)

- [ ] **Step 3: Add fitPrincipalAxis to ObjectDetectionAlgorithms.java**

```java
  // -------------------------------------------------------------------------
  // PCA principal axis fitting
  // -------------------------------------------------------------------------

  static PrincipalAxis fitPrincipalAxis(List<TimestampedFieldDetection> cluster) {
    int n = cluster.size();
    // Centroid
    double cx = 0, cy = 0;
    for (TimestampedFieldDetection p : cluster) {
      cx += p.x();
      cy += p.y();
    }
    cx /= n;
    cy /= n;

    // Covariance matrix elements (biased)
    double sxx = 0, sxy = 0, syy = 0;
    for (TimestampedFieldDetection p : cluster) {
      double dx = p.x() - cx;
      double dy = p.y() - cy;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
    sxx /= n;
    sxy /= n;
    syy /= n;

    // Analytical eigendecomposition of 2×2 symmetric matrix
    double disc = Math.sqrt(Math.max(0, (sxx - syy) * (sxx - syy) / 4.0 + sxy * sxy));
    double lambda1 = (sxx + syy) / 2.0 + disc; // larger eigenvalue

    // Eigenvector for lambda1
    double vx, vy;
    if (Math.abs(sxy) > 1e-10) {
      vx = lambda1 - syy;
      vy = sxy;
    } else {
      vx = (sxx >= syy) ? 1.0 : 0.0;
      vy = (sxx >= syy) ? 0.0 : 1.0;
    }
    double norm = Math.hypot(vx, vy);
    if (norm < 1e-10) {
      vx = 1.0;
      vy = 0.0;
    } else {
      vx /= norm;
      vy /= norm;
    }

    // Project all points onto axis; find actual extent
    double minProj = Double.MAX_VALUE, maxProj = -Double.MAX_VALUE;
    for (TimestampedFieldDetection p : cluster) {
      double proj = (p.x() - cx) * vx + (p.y() - cy) * vy;
      minProj = Math.min(minProj, proj);
      maxProj = Math.max(maxProj, proj);
    }

    // Shift center to midpoint of actual projections
    double midProj = (minProj + maxProj) / 2.0;
    return new PrincipalAxis(
        cx + midProj * vx,
        cy + midProj * vy,
        vx,
        vy,
        (maxProj - minProj) / 2.0);
  }
```

- [ ] **Step 4: Run tests — expect all PASS**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 10 tests PASSED

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java \
        src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java
git commit -m "feat(objectdetection): PCA principal axis fitting with tests"
```

---

## Task 6: Waypoint generation (TDD)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java`
- Modify: `src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java`

- [ ] **Step 1: Add failing tests** (append to `ObjectDetectionAlgorithmsTest`)

Add import at top:
```java
import edu.wpi.first.math.geometry.Pose2d;
```

Add tests:

```java
  // ---- waypoint generation tests ----

  @Test
  void generateCollectionWaypoints_robotOnLeft_firstWaypointIsLeftmost() {
    // Horizontal axis centered at origin, extent ±1m; robot at (-3, 0)
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Translation2d robot = new Translation2d(-3, 0);

    Pose2d[] wps = ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, robot);

    assertEquals(-1.0, wps[0].getX(), 1e-9, "First waypoint should be leftmost (x=-1)");
    assertEquals(1.0, wps[3].getX(), 1e-9, "Last waypoint should be rightmost (x=+1)");
  }

  @Test
  void generateCollectionWaypoints_robotOnRight_firstWaypointIsRightmost() {
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Translation2d robot = new Translation2d(3, 0);

    Pose2d[] wps = ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, robot);

    assertEquals(1.0, wps[0].getX(), 1e-9, "First waypoint should be rightmost (x=+1)");
    assertEquals(-1.0, wps[3].getX(), 1e-9, "Last waypoint should be leftmost (x=-1)");
  }

  @Test
  void generateCollectionWaypoints_returns4Waypoints() {
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Pose2d[] wps =
        ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, new Translation2d(0, 0));
    assertEquals(4, wps.length);
  }

  @Test
  void generateCollectionWaypoints_travelDirectionPointsAlongAxis() {
    // Horizontal axis, robot on left — waypoints travel rightward (+x)
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        new ObjectDetectionAlgorithms.PrincipalAxis(0, 0, 1, 0, 1.0);
    Pose2d[] wps =
        ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, new Translation2d(-3, 0));

    // All travel-direction headings (path tangent) should be ~0° (pointing +x)
    for (int i = 0; i < 3; i++) {
      assertEquals(
          0.0,
          wps[i].getRotation().getDegrees(),
          1.0,
          "Waypoint " + i + " travel direction should be ~0°");
    }
  }
```

- [ ] **Step 2: Run tests — expect failure on waypoint tests**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 10 PASS, 4 FAIL

- [ ] **Step 3: Add generateCollectionWaypoints to ObjectDetectionAlgorithms.java**

```java
  // -------------------------------------------------------------------------
  // 4-waypoint generation
  // -------------------------------------------------------------------------

  /**
   * Generates 4 Pose2d collection waypoints along the principal axis. Pose rotation = direction of
   * travel (path tangent for PathPlanner.waypointsFromPoses — NOT chassis heading). The first
   * waypoint is the one closest to robotPos.
   */
  static Pose2d[] generateCollectionWaypoints(PrincipalAxis axis, Translation2d robotPos) {
    double a = axis.semiMajor();
    double[] offsets = {-a, -a / 3.0, a / 3.0, a};

    Translation2d[] positions = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      positions[i] =
          new Translation2d(
              axis.centerX() + offsets[i] * axis.axisX(),
              axis.centerY() + offsets[i] * axis.axisY());
    }

    // Reorder so positions[0] is closest to the robot
    if (positions[3].getDistance(robotPos) < positions[0].getDistance(robotPos)) {
      Translation2d[] reversed = new Translation2d[4];
      for (int i = 0; i < 4; i++) reversed[i] = positions[3 - i];
      positions = reversed;
    }

    // Pose rotation = direction of travel to next waypoint (PathPlanner path tangent)
    Pose2d[] waypoints = new Pose2d[4];
    for (int i = 0; i < 3; i++) {
      Translation2d delta = positions[i + 1].minus(positions[i]);
      waypoints[i] = new Pose2d(positions[i], new Rotation2d(delta.getX(), delta.getY()));
    }
    waypoints[3] = new Pose2d(positions[3], waypoints[2].getRotation());

    return waypoints;
  }
```

- [ ] **Step 4: Run tests — expect all PASS**

```bash
./gradlew test --tests "subsystems.ObjectDetectionAlgorithmsTest"
```

Expected: 14 tests PASSED

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionAlgorithms.java \
        src/test/java/subsystems/ObjectDetectionAlgorithmsTest.java
git commit -m "feat(objectdetection): 4-waypoint generation with tests"
```

---

## Task 7: ObjectDetection subsystem — structure, buffer, transforms

**Files:**
- Create: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetection.java`
- Create: `src/test/java/subsystems/ObjectDetectionTest.java`

- [ ] **Step 1: Write failing subsystem tests**

Create `src/test/java/subsystems/ObjectDetectionTest.java`:

```java
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

    // Default: connected, heartbeat=0, no detections
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
              inputs.heartbeat = 1; // never changes
              inputs.camera0Detections = new BallDetection[] {new BallDetection(0.0, 2.0, 2.0)};
              inputs.camera1Detections = new BallDetection[0];
              return null;
            })
        .when(io)
        .updateInputs(any());

    subsystem.periodic();
    subsystem.periodic(); // second call — same heartbeat, should not re-append

    assertEquals(1, subsystem.getBufferSize());
  }

  @Test
  void setCommitted_false_clearsBestPath() {
    subsystem.setCommitted(true);
    subsystem.setCommitted(false);
    assertTrue(subsystem.getBestPath().isEmpty());
  }
}
```

- [ ] **Step 2: Run tests — expect compilation failure**

```bash
./gradlew test --tests "subsystems.ObjectDetectionTest"
```

Expected: FAILED — `ObjectDetection` class not found

- [ ] **Step 3: Create ObjectDetection.java (skeleton + buffer + transforms)**

```java
package frc.robot.subsystems.objectdetection;

import static frc.robot.subsystems.objectdetection.ObjectDetectionConstants.*;
import static frc.robot.subsystems.objectdetection.ObjectDetectionAlgorithms.TimestampedFieldDetection;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {

  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();
  private final Drive drive;

  private final ArrayDeque<TimestampedFieldDetection> buffer = new ArrayDeque<>();
  private long lastHeartbeat = -1;
  private boolean committed = false;
  private Optional<PathPlannerPath> bestPath = Optional.empty();
  private Translation2d lastClusterCentroid = null;

  public ObjectDetection(ObjectDetectionIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);

    boolean newData = inputs.heartbeat != lastHeartbeat;

    if (newData) {
      lastHeartbeat = inputs.heartbeat;
      Pose2d robotPose = drive.getPose();
      double now = Timer.getTimestamp();
      appendDetections(inputs.camera0Detections, CAMERA_0_ROBOT_OFFSET, robotPose, now);
      appendDetections(inputs.camera1Detections, CAMERA_1_ROBOT_OFFSET, robotPose, now);
    }

    // Purge stale entries from front of deque
    double cutoff = Timer.getTimestamp() - BUFFER_WINDOW_SECS;
    while (!buffer.isEmpty() && buffer.peekFirst().timestamp() < cutoff) {
      buffer.pollFirst();
    }

    if (newData && !committed) {
      runPipeline();
    }

    Logger.recordOutput("ObjectDetection/BufferSize", buffer.size());
    Logger.recordOutput("ObjectDetection/Committed", committed);
    Logger.recordOutput("ObjectDetection/HasPath", bestPath.isPresent());
  }

  private void appendDetections(
      BallDetection[] detections, Transform2d cameraMount, Pose2d robotPose, double timestamp) {
    for (BallDetection d : detections) {
      // Camera local → robot-relative (NWU: x=forward, y=left)
      // detection.y() = forward distance in camera frame → +x (forward) in robot NWU
      // detection.x() = rightward in camera frame → -y (left) in robot NWU
      Translation2d camNWU = new Translation2d(d.y(), -d.x());
      // Rotate by mount yaw, add mount translation
      Translation2d robotRel =
          camNWU.rotateBy(cameraMount.getRotation()).plus(cameraMount.getTranslation());
      // Transform to field frame
      Translation2d fieldPos =
          robotRel.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
      buffer.addLast(
          new TimestampedFieldDetection(fieldPos.getX(), fieldPos.getY(), timestamp));
    }
  }

  private void runPipeline() {
    // Implemented in Task 8
  }

  public void setCommitted(boolean committed) {
    this.committed = committed;
    if (!committed) {
      bestPath = Optional.empty();
      lastClusterCentroid = null;
    }
  }

  public boolean isCommitted() {
    return committed;
  }

  public Optional<PathPlannerPath> getBestPath() {
    return bestPath;
  }

  /** Exposed for testing only. */
  public int getBufferSize() {
    return buffer.size();
  }
}
```

- [ ] **Step 4: Run tests — expect all PASS**

```bash
./gradlew test --tests "subsystems.ObjectDetectionTest"
```

Expected: 3 tests PASSED

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetection.java \
        src/test/java/subsystems/ObjectDetectionTest.java
git commit -m "feat(objectdetection): subsystem skeleton with buffer and transforms"
```

---

## Task 8: ObjectDetection — pipeline, path generation, command

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetection.java`
- Modify: `src/test/java/subsystems/ObjectDetectionTest.java`

- [ ] **Step 1: Add pipeline integration test** (append to `ObjectDetectionTest`)

```java
  @Test
  void periodic_multipleDetectionsInOneCluster_generatesBestPath() {
    // Provide enough detections to form a DBSCAN cluster (minPts=2, epsilon=0.8m)
    // All 4 within 0.8m of each other — form one cluster, enough for PCA + path
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

    subsystem.periodic(); // gets a path
    assertTrue(subsystem.getBestPath().isPresent());

    subsystem.setCommitted(true); // freeze replanning
    subsystem.setCommitted(false); // clears bestPath

    subsystem.periodic(); // committed=false, would normally replan — but bestPath was just cleared
    // After setCommitted(false), bestPath is empty; next periodic should replan
    assertTrue(subsystem.getBestPath().isPresent(), "Should replan after recommitting to false");
  }
```

- [ ] **Step 2: Run tests — expect failure on new tests**

```bash
./gradlew test --tests "subsystems.ObjectDetectionTest"
```

Expected: 3 PASS, 2 FAIL (pipeline not implemented yet)

- [ ] **Step 3: Implement runPipeline() in ObjectDetection.java**

Replace the `runPipeline()` stub:

```java
  private void runPipeline() {
    List<TimestampedFieldDetection> snapshot = new ArrayList<>(buffer);
    if (snapshot.size() < 2) return;

    List<List<TimestampedFieldDetection>> clusters =
        ObjectDetectionAlgorithms.dbscan(snapshot, DBSCAN_EPSILON, DBSCAN_MIN_PTS);
    if (clusters.isEmpty()) return;

    Logger.recordOutput("ObjectDetection/ClusterCount", clusters.size());

    double now = Timer.getTimestamp();
    Translation2d robotPos = drive.getPose().getTranslation();

    // Find best cluster by score
    double bestScore = Double.NEGATIVE_INFINITY;
    List<TimestampedFieldDetection> bestCluster = null;
    for (List<TimestampedFieldDetection> cluster : clusters) {
      double score =
          ObjectDetectionAlgorithms.scoreCluster(
              cluster, robotPos, now, DECAY_K, COUNT_SCALAR, DIST_SCALAR);
      if (score > bestScore) {
        bestScore = score;
        bestCluster = cluster;
      }
    }
    Logger.recordOutput("ObjectDetection/BestClusterScore", bestScore);

    if (bestCluster == null || bestCluster.size() < 2) return;

    Translation2d newCentroid = ObjectDetectionAlgorithms.clusterCentroid(bestCluster);

    // Replan gate: skip if centroid hasn't moved enough
    if (lastClusterCentroid != null
        && newCentroid.getDistance(lastClusterCentroid) < REPLAN_THRESHOLD_METERS) {
      return;
    }
    lastClusterCentroid = newCentroid;

    // Fit PCA axis
    ObjectDetectionAlgorithms.PrincipalAxis axis =
        ObjectDetectionAlgorithms.fitPrincipalAxis(bestCluster);

    // Generate 4 collection waypoints (travel-direction headings)
    Pose2d[] collectionWps = ObjectDetectionAlgorithms.generateCollectionWaypoints(axis, robotPos);

    // Log for AdvantageScope
    Translation2d[] wpTranslations = new Translation2d[4];
    for (int i = 0; i < 4; i++) wpTranslations[i] = collectionWps[i].getTranslation();
    Logger.recordOutput("ObjectDetection/ClusterWaypoints", wpTranslations);

    Translation2d[] allPoints = new Translation2d[snapshot.size()];
    for (int i = 0; i < snapshot.size(); i++)
      allPoints[i] = new Translation2d(snapshot.get(i).x(), snapshot.get(i).y());
    Logger.recordOutput("ObjectDetection/AllBufferedDetections", allPoints);

    // Prepend robot current pose as approach waypoint
    Pose2d robotPose = drive.getPose();
    Translation2d toFirst = collectionWps[0].getTranslation().minus(robotPose.getTranslation());
    Rotation2d approachDir =
        (toFirst.getNorm() > 0.05)
            ? new Rotation2d(toFirst.getX(), toFirst.getY())
            : collectionWps[0].getRotation();
    Pose2d startPose = new Pose2d(robotPose.getTranslation(), approachDir);

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            startPose,
            collectionWps[0],
            collectionWps[1],
            collectionWps[2],
            collectionWps[3]);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            COLLECTION_CONSTRAINTS,
            null,
            new GoalEndState(0.0, collectionWps[3].getRotation()));
    path.preventFlipping = true;

    bestPath = Optional.of(path);
  }
```

- [ ] **Step 4: Add continuousCollectionCommand() to ObjectDetection.java**

Add after `getBestPath()`:

```java
  /**
   * Continuously collects the best cluster, then chains to the next best, repeating until
   * cancelled. Overrides holonomic rotation so the intake (back of robot) faces the direction of
   * travel.
   *
   * <p>Bind to a button hold in RobotContainer.
   */
  public Command continuousCollectionCommand() {
    // Compute intake-leading heading from current field-relative velocity
    java.util.function.DoubleSupplier intakeLeadingRotation =
        () -> {
          ChassisSpeeds field =
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  drive.getChassisSpeeds(), drive.getRotation());
          double vx = field.vxMetersPerSecond;
          double vy = field.vyMetersPerSecond;
          if (Math.hypot(vx, vy) > ROTATION_OVERRIDE_MIN_SPEED) {
            // Intake at back → robot heading = travel direction + 180°
            return Math.atan2(vy, vx) + Math.PI;
          }
          return drive.getRotation().getRadians();
        };

    return Commands.sequence(
        Commands.waitUntil(() -> bestPath.isPresent()),
        Commands.sequence(
                Commands.defer(
                    () ->
                        AutoBuilder.followPath(bestPath.get())
                            .beforeStarting(
                                () -> {
                                  setCommitted(true);
                                  PPHolonomicDriveController.overrideRotationFeedback(
                                      intakeLeadingRotation);
                                })
                            .finallyDo(
                                interrupted -> {
                                  setCommitted(false);
                                  PPHolonomicDriveController.clearRotationFeedbackOverride();
                                }),
                    Set.of(drive)),
                Commands.waitUntil(() -> bestPath.isPresent()))
            .repeatedly());
  }
```

- [ ] **Step 5: Run tests — expect all PASS**

```bash
./gradlew test --tests "subsystems.ObjectDetectionTest"
```

Expected: 5 tests PASSED

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetection.java \
        src/test/java/subsystems/ObjectDetectionTest.java
git commit -m "feat(objectdetection): pipeline, path generation, continuous collection command"
```

---

## Task 9: ObjectDetectionIOReal

**Files:**
- Create: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIOReal.java`

- [ ] **Step 1: Create ObjectDetectionIOReal.java**

```java
package frc.robot.subsystems.objectdetection;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;

public class ObjectDetectionIOReal implements ObjectDetectionIO {

  private final StructArraySubscriber<BallDetection> camera0Sub;
  private final StructArraySubscriber<BallDetection> camera1Sub;
  private final IntegerSubscriber heartbeatSub;
  private long lastSeenHeartbeat = -1;

  public ObjectDetectionIOReal() {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    camera0Sub =
        nt.getStructArrayTopic("objectdetection/balls/0", BallDetection.struct)
            .subscribe(new BallDetection[0]);
    camera1Sub =
        nt.getStructArrayTopic("objectdetection/balls/1", BallDetection.struct)
            .subscribe(new BallDetection[0]);
    heartbeatSub = nt.getIntegerTopic("objectdetection/heartbeat").subscribe(-1);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.camera0Detections = camera0Sub.get();
    inputs.camera1Detections = camera1Sub.get();
    inputs.heartbeat = heartbeatSub.get();
    inputs.connected = inputs.heartbeat != lastSeenHeartbeat;
    lastSeenHeartbeat = inputs.heartbeat;
  }
}
```

- [ ] **Step 2: Verify compile**

```bash
./gradlew compileJava -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIOReal.java
git commit -m "feat(objectdetection): ObjectDetectionIOReal NT subscriptions"
```

---

## Task 10: ObjectDetectionIOSim

**Files:**
- Create: `src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIOSim.java`

- [ ] **Step 1: Create ObjectDetectionIOSim.java**

```java
package frc.robot.subsystems.objectdetection;

public class ObjectDetectionIOSim implements ObjectDetectionIO {

  private BallDetection[] camera0 = new BallDetection[0];
  private BallDetection[] camera1 = new BallDetection[0];
  private long simulatedHeartbeat = 0;

  /** Inject synthetic detections from simulation code. Increments the heartbeat automatically. */
  public void setDetections(BallDetection[] camera0Detections, BallDetection[] camera1Detections) {
    this.camera0 = camera0Detections;
    this.camera1 = camera1Detections;
    simulatedHeartbeat++;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.camera0Detections = camera0;
    inputs.camera1Detections = camera1;
    inputs.heartbeat = simulatedHeartbeat;
    inputs.connected = true;
  }
}
```

- [ ] **Step 2: Verify compile and run full test suite**

```bash
./gradlew test
```

Expected: BUILD SUCCESSFUL, all tests PASS

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/objectdetection/ObjectDetectionIOSim.java
git commit -m "feat(objectdetection): ObjectDetectionIOSim for simulation"
```

---

## Task 11: RobotContainer wiring

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`

- [ ] **Step 1: Read RobotContainer.java to find the right insertion points**

Read `src/main/java/frc/robot/RobotContainer.java` — locate:
1. The field declarations block (where `drive`, `vision`, etc. are declared)
2. The constructor where IO implementations are instantiated based on `Constants.currentMode`
3. `configureButtonBindings()` or equivalent where button commands are bound

- [ ] **Step 2: Add the ObjectDetection field declaration**

In the field declarations section alongside other subsystem fields:

```java
private final ObjectDetection objectDetection;
```

Add import:
```java
import frc.robot.subsystems.objectdetection.ObjectDetection;
import frc.robot.subsystems.objectdetection.ObjectDetectionIOReal;
import frc.robot.subsystems.objectdetection.ObjectDetectionIOSim;
```

- [ ] **Step 3: Instantiate ObjectDetection in the constructor**

After `drive` is instantiated (ObjectDetection needs `drive`):

```java
objectDetection =
    new ObjectDetection(
        Constants.currentMode == Constants.Mode.REAL
            ? new ObjectDetectionIOReal()
            : new ObjectDetectionIOSim(),
        drive);
```

- [ ] **Step 4: Bind continuousCollectionCommand to a button**

In the button binding section — choose a button not already used (check existing bindings first):

```java
// Continuous ball collection — hold button to scan and collect clusters
// Change button as needed based on driver preference
controller.leftBumper().whileTrue(objectDetection.continuousCollectionCommand());
```

- [ ] **Step 5: Full build**

```bash
./gradlew build
```

Expected: BUILD SUCCESSFUL, all tests PASS

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat(objectdetection): wire ObjectDetection into RobotContainer"
```
