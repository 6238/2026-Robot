# Object Detection Subsystem Design

**Date:** 2026-04-05
**Project:** FRC Team 6238 — 2026 Robot
**Feature:** Ball (fuel) detection, clustering, and autonomous collection path generation

---

## Overview

A new `ObjectDetection` subsystem reads ball detections published by the Jetson vision pipeline over NetworkTables, accumulates them in a field-relative rolling buffer, clusters them with DBSCAN, scores clusters, fits an ellipse to the best cluster, and generates a 4-waypoint PathPlanner path for the robot to collect along. Path generation is gated to avoid unnecessary recomputation and GC pressure. Once the robot begins executing a collection path, replanning is frozen until the path completes or is interrupted.

---

## File Structure

```
src/main/java/frc/robot/subsystems/objectdetection/
  BallDetection.java                // struct record matching NT_FORMAT.md schema
  ObjectDetectionIO.java            // AK IO interface + @AutoLog inputs
  ObjectDetectionIOReal.java        // NT subscribers for both cameras + heartbeat
  ObjectDetectionIOSim.java         // synthetic detections for simulation/testing
  ObjectDetectionConstants.java     // all tunable parameters
  ObjectDetection.java              // subsystem — all algorithm logic
```

---

## Architecture

Follows the existing AdvantageKit IO layer pattern used by Vision, Shooter, etc.

### `ObjectDetectionIO` (interface)

Inputs exposed to the subsystem via `@AutoLog ObjectDetectionIOInputs`:

| Field | Type | Description |
|---|---|---|
| `camera0Detections` | `BallDetection[]` | Latest frame from left camera |
| `camera1Detections` | `BallDetection[]` | Latest frame from right camera |
| `heartbeat` | `long` | Increments each processed batch (~30 Hz) |
| `connected` | `boolean` | True if heartbeat changed this loop |

### `ObjectDetectionIOReal`

Subscribes to `objectdetection/balls/0`, `objectdetection/balls/1`, and `objectdetection/heartbeat` via `StructArraySubscriber<BallDetection>` and `IntegerSubscriber`. Sets `connected` by comparing heartbeat to previous value.

### `ObjectDetectionIOSim`

Injects configurable synthetic `BallDetection[]` arrays for unit testing and simulation. Increments a fake heartbeat each loop.

### `ObjectDetection` (subsystem)

Holds:
- `Drive drive` — for `drive.getPose()` during coordinate transforms
- `Deque<TimestampedFieldDetection> buffer` — rolling field-relative detection store
- `long lastHeartbeat` — change detection gate
- `Optional<PathPlannerPath> bestPath` — most recently generated path
- `Translation2d lastClusterCentroid` — replan threshold check
- `boolean committed` — frozen-replan flag

---

## Coordinate Transforms

**Chain per detection (camera-local → field):**

```
camera-local (x right, y forward)
  → robot-relative  via CAMERA_N_ROBOT_OFFSET (Transform2d constant)
  → field-relative  via drive.getPose() at time of NT frame receipt
  → stored as TimestampedFieldDetection(fieldX, fieldY, captureTimestamp)
```

Robot-frame transform (standard 2D rigid body):
```
robotX = mountX + detection.y * cos(mountYaw) - detection.x * sin(mountYaw)
robotY = mountY + detection.y * sin(mountYaw) + detection.x * cos(mountYaw)
```

Camera mount constants (placeholders — fill in from robot CAD):
```java
static final Transform2d CAMERA_0_ROBOT_OFFSET = new Transform2d(0.0, 0.15, Rotation2d.fromDegrees(0));
static final Transform2d CAMERA_1_ROBOT_OFFSET = new Transform2d(0.0, -0.15, Rotation2d.fromDegrees(0));
```

---

## Rolling Buffer

- **Type:** `ArrayDeque<TimestampedFieldDetection>` (each entry: fieldX, fieldY, captureTimestamp)
- **Max age:** `BUFFER_WINDOW_SECS` (default 1.5s) — entries older than this are purged at the start of each `periodic()` before DBSCAN runs
- **No max count cap** — at 30 Hz × 2 cameras × ~20 detections, worst-case ~1800 entries; all trivially fast for DBSCAN at these counts
- **Buffer cleared** when `setCommitted(true)` is called — prevents stale field positions from contaminating the next replan after a collection run

---

## Algorithm Pipeline

Runs in `periodic()` only when:
1. `inputs.heartbeat != lastHeartbeat` (new data from Jetson)
2. `!committed` (not mid-execution)

### Step 1 — DBSCAN Clustering

- Input: all buffer entries after age purge, as (fieldX, fieldY) points
- Algorithm: naive O(n²) — sufficient for n < 200
- Parameters:
  - `DBSCAN_EPSILON` — neighborhood radius in meters (default 0.8m)
  - `DBSCAN_MIN_PTS` — minimum points to form a cluster (default 2)
- Output: `List<List<TimestampedFieldDetection>>` — noise points discarded

### Step 2 — Cluster Scoring

```
score = (weightedCount * COUNT_SCALAR) - (distSquared * DIST_SCALAR)
```

- `weightedCount = Σ exp(-DECAY_K * age_seconds)` over all detections in cluster
  - Rewards recency: a cluster of 5 balls seen 0.1s ago beats 7 balls seen 1.4s ago
- `distSquared` = squared distance (meters²) from current robot pose to cluster centroid
- Both scalars are tunable constants; higher score = better cluster
- Pick `argmax` across all clusters

Parameters:
| Constant | Default | Description |
|---|---|---|
| `COUNT_SCALAR` | 1.0 | Weight of effective detection count in score |
| `DIST_SCALAR` | 0.5 | Weight of distance penalty in score |
| `DECAY_K` | 2.0 | Exponential age decay rate (per second) |

### Step 3 — Ellipse Fitting (Fitzgibbon Direct Method)

- Solves 6-parameter conic `Ax² + Bxy + Cy² + Dx + Ey + F = 0` constrained to ellipse
- Extracts: center (cx, cy), semi-axes (a ≥ b), orientation angle θ
- Degenerate cases:
  - **< 5 points:** fall back to line fit — treat as ellipse with b=0, a = half the point spread along the principal axis
  - **< 2 points:** no path generated, `bestPath` remains empty

### Step 4 — 4-Waypoint Generation

- Major axis vector: `(cos θ, sin θ)`
- Waypoint positions along major axis from center: `-a`, `-a/3`, `+a/3`, `+a`
- Reorder so waypoint[0] is closest to the current robot pose
- **Heading per waypoint** = direction toward next waypoint + 180°
  - Intake is at the back of the robot; robot must face away from its direction of travel so the intake leads
- Final waypoint heading = same as penultimate (continue same direction)

### Step 5 — PathPlanner Path Generation

```java
List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(p0, p1, p2, p3);
PathPlannerPath path = new PathPlannerPath(
    waypoints,
    COLLECTION_CONSTRAINTS,
    null,
    new GoalEndState(0.0, lastHeading)
);
path.preventFlipping = true;
```

- `COLLECTION_CONSTRAINTS`: `PathConstraints(MAX_SPEED_MPS, MAX_ACCEL_MPS2, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL)` — default 2.0 m/s, 3.0 m/s²
- `GoalEndState(0.0, ...)` — robot stops at end of cluster

**Replan gate:** Before accepting a newly generated path, compare new cluster centroid to `lastClusterCentroid`. If distance < `REPLAN_THRESHOLD_METERS` (default 0.15m), discard new path and keep existing. Prevents path churn when cluster is spatially stable. `lastClusterCentroid` is updated only when a new path is accepted (i.e. the threshold was exceeded).

---

## State Machine

| State | Behavior |
|---|---|
| `SCANNING` | Pipeline runs every heartbeat tick; `bestPath` updated when cluster changes |
| `COMMITTED` | Pipeline frozen; `bestPath` locked; buffer cleared |

Transitions:
- `SCANNING → COMMITTED`: `setCommitted(true)` called when command starts following the path
- `COMMITTED → SCANNING`: `setCommitted(false)` called when path finishes or is interrupted

---

## Command Interface

`ObjectDetection` exposes a `followBestClusterCommand(Drive drive)` factory:

```java
public Command followBestClusterCommand(Drive drive) {
    // Commands.defer captures bestPath at scheduling time, not construction time.
    return Commands.defer(
        () -> {
            if (bestPath.isEmpty()) return Commands.none();
            return AutoBuilder.followPath(bestPath.get())
                .beforeStarting(() -> setCommitted(true))
                .finallyDo(interrupted -> setCommitted(false));
        },
        Set.of(drive));
}
```

Called from `RobotContainer` on a button binding. If no path is available when triggered, the command does nothing. `Commands.defer` ensures the path is captured at the moment the button is pressed, not when the command object is constructed.

---

## AK Logging

All logged in `ObjectDetection.periodic()`:

| Key | Type | Description |
|---|---|---|
| `ObjectDetection/BufferSize` | int | Current entry count in rolling buffer |
| `ObjectDetection/ClusterCount` | int | Number of DBSCAN clusters this tick |
| `ObjectDetection/BestClusterScore` | double | Score of the selected cluster |
| `ObjectDetection/HasPath` | boolean | Whether `bestPath` is present |
| `ObjectDetection/Committed` | boolean | Whether replanning is frozen |
| `ObjectDetection/ClusterWaypoints` | `Translation2d[]` | 4 waypoints of current path (AdvantageScope) |
| `ObjectDetection/AllBufferedDetections` | `Translation2d[]` | All field-relative points in buffer |

---

## Constants Summary (`ObjectDetectionConstants`)

| Constant | Default | Description |
|---|---|---|
| `CAMERA_0_ROBOT_OFFSET` | `(0, 0.15, 0°)` | Left camera mount (placeholder) |
| `CAMERA_1_ROBOT_OFFSET` | `(0, -0.15, 0°)` | Right camera mount (placeholder) |
| `BUFFER_WINDOW_SECS` | 1.5 | Max age of buffered detections |
| `DBSCAN_EPSILON` | 0.8 | Cluster neighborhood radius (m) |
| `DBSCAN_MIN_PTS` | 2 | Min points to form a cluster |
| `COUNT_SCALAR` | 1.0 | Cluster score weight for detection count |
| `DIST_SCALAR` | 0.5 | Cluster score penalty for distance |
| `DECAY_K` | 2.0 | Exponential age decay rate (per second) |
| `REPLAN_THRESHOLD_METERS` | 0.15 | Min centroid shift to trigger new path |
| `MAX_COLLECTION_SPEED_MPS` | 2.0 | PathPlanner max speed for collection |
| `MAX_COLLECTION_ACCEL_MPS2` | 3.0 | PathPlanner max acceleration |
