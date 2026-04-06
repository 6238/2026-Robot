# NetworkTables Output Format

Published by the Jetson vision pipeline (`ds_nt_probe`) to the roboRIO over NT4.

---

## Topics

| Topic | NT Type | WPILib Type | Description |
|-------|---------|-------------|-------------|
| `objectdetection/balls/0` | `raw` | `StructArrayTopic<BallDetection>` | Detections from left camera |
| `objectdetection/balls/1` | `raw` | `StructArrayTopic<BallDetection>` | Detections from right camera |
| `objectdetection/heartbeat` | `int` | `IntegerTopic` | Increments every processed batch (~30 Hz); use to detect a dead pipeline |
| `/.schema/struct:BallDetection` | `structschema` | *(auto)* | Schema announcement consumed automatically by WPILib |

---

## BallDetection Struct

**Schema string:** `double x;double y;double distance`

**Wire format:** little-endian IEEE 754 doubles, packed with no padding.

| Field | Type | Bytes | Description |
|-------|------|-------|-------------|
| `x` | `double` | 0–7 | Lateral offset in metres. Right of camera centre is **positive**. |
| `y` | `double` | 8–15 | Forward distance in metres. Always **positive** (ball is in front of camera). |
| `distance` | `double` | 16–23 | 3-D Euclidean distance from camera to ball in metres: `sqrt(x² + y² + h²)` |

Each detection is **24 bytes**. An empty frame publishes **0 bytes** (empty array).

### Java record

```java
package frc.robot.subsystems.objectdetection;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

public record BallDetection(double x, double y, double distance) implements StructSerializable {

  public static final BallDetectionStruct struct = new BallDetectionStruct();

  public static final class BallDetectionStruct implements Struct<BallDetection> {
    @Override public Class<BallDetection> getTypeClass() { return BallDetection.class; }
    @Override public String getTypeName()   { return "BallDetection"; }
    @Override public String getTypeString() { return "struct:BallDetection"; }
    @Override public int    getSize()       { return kSizeDouble * 3; }
    @Override public String getSchema()     { return "double x;double y;double distance"; }

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

### Subscribing in Java

```java
StructArraySubscriber<BallDetection> sub = NetworkTableInstance.getDefault()
    .getStructArrayTopic("objectdetection/balls/0", BallDetection.struct)
    .subscribe(new BallDetection[]{});

// In periodic:
BallDetection[] balls = sub.get();
```

---

## Coordinate Frame

The coordinate origin is the camera's optical centre projected onto the floor.

```
        Camera
          |  (height h)
          |
    ------+------ floor (z = 0)
          |
     x <--+---> x
    (left)   (right)
          |
          v  y (forward)
```

- **x**: positive to the right of the camera
- **y**: positive away from the camera (always > 0 for a detected ball)
- **distance**: straight-line distance in 3-D space, `sqrt(x² + y² + h²)`

---

## Projection Model

Bounding box centres are back-projected to the floor plane (`z = 0`) using the
pinhole camera model and calibrated intrinsics from `L.json` / `R.json`.

```
alpha_x  = atan2(px − cx,  fx)     # lateral ray angle
alpha_y  = atan2(py − cy,  fy)     # vertical ray angle (down = positive)
theta_v  = cam_pitch + alpha_y     # total angle below horizontal

y        = cam_height / tan(theta_v)
x        = y * tan(alpha_x)
distance = sqrt(x² + y² + cam_height²)
```

Detections where `theta_v ≤ 0.01 rad` (ray does not reach the floor) are silently
dropped and do not appear in the published array.

---

## Camera Calibration

Intrinsics from OpenCV calibration at 1280×720, scaled to pipeline resolution (640×360):

| Camera | Source | fx (pipe) | fy (pipe) | cx (pipe) | cy (pipe) |
|--------|--------|-----------|-----------|-----------|-----------|
| 0 (left)  | `L.json` | 454.37 | 454.85 | 354.83 | 149.33 |
| 1 (right) | `R.json` | 452.41 | 452.79 | 350.62 | 148.78 |

Mounting parameters (set via environment variables):

| Variable | Default | Description |
|----------|---------|-------------|
| `CAM0_HEIGHT` | `0.5` | Camera height above floor, metres |
| `CAM0_PITCH_DEG` | `20.0` | Camera tilt below horizontal, degrees |
| `CAM1_HEIGHT` | `0.5` | Same for right camera |
| `CAM1_PITCH_DEG` | `20.0` | Same for right camera |

---

## Heartbeat

`objectdetection/heartbeat` is an integer that increments once per processed
batch (approximately every 33 ms at 30 fps). On the robot, compare the last seen
value against the current value in `robotPeriodic` to detect a stale or crashed
pipeline:

```java
long lastHeartbeat = -1;

boolean isCameraAlive() {
    long hb = heartbeatSub.get();
    boolean alive = hb != lastHeartbeat;
    lastHeartbeat = hb;
    return alive;
}
```
