package frc.robot.subsystems.objectdetection;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.wpilibj.Timer;

public class ObjectDetectionIOJetson implements ObjectDetectionIO {
  private final IntegerSubscriber heartbeat;
  private final StructArraySubscriber<TargetDetection> observationsSub;

  private double heartbeatChangeTime = 0.0;
  private long prevHeartbeatValue = 0;
  private long lastNtTimestamp = 0;

  public ObjectDetectionIOJetson() {
    NetworkTable jetsonTable =
        NetworkTableInstance.getDefault().getTable(ObjectDetectionConstants.JetsonTable);

    heartbeat =
        jetsonTable
            .getIntegerTopic("heartbeat")
            .subscribe(ObjectDetectionConstants.UNCONNECTED_HEARTBEAT_VALUE);

    observationsSub =
        jetsonTable
            .getStructArrayTopic("observations", TargetDetection.struct)
            .subscribe(new TargetDetection[0]);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.connected = isConnected();
    inputs.targetDetections = new TargetDetection[0];

    TimestampedObject<TargetDetection[]> atomicData = observationsSub.getAtomic();

    if (atomicData.timestamp == lastNtTimestamp) {
      return;
    }

    lastNtTimestamp = atomicData.timestamp;

    if (atomicData.value.length > 0) {
      inputs.targetDetections = atomicData.value.clone();
    }
  }

  private boolean isConnected() {
    long cur = heartbeat.get();
    double now = Timer.getFPGATimestamp();

    if (cur <= ObjectDetectionConstants.UNCONNECTED_HEARTBEAT_VALUE) {
      return false;
    }

    if (cur != prevHeartbeatValue) {
      prevHeartbeatValue = cur;
      heartbeatChangeTime = now;
    }

    return (now - heartbeatChangeTime) < ObjectDetectionConstants.HEARTBEAT_TOLERANCE;
  }
}
