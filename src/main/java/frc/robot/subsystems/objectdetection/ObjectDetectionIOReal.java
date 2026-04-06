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
