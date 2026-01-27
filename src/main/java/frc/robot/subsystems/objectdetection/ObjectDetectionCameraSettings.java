package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Transform3d;

public class ObjectDetectionCameraSettings {
    public String cameraName;
    public Transform3d robotToCamera;

    public ObjectDetectionCameraSettings() {
        cameraName = "";
        robotToCamera = new Transform3d();
    }

    public ObjectDetectionCameraSettings(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
    }
}
