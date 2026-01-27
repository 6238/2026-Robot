package frc.robot.subsystems.objectdetection;

import edu.wpi.first.math.geometry.Transform3d;

public class ObjectDetectionConstants {
    public ObjectDetectionCameraSettings intakeCamera = new ObjectDetectionCameraSettings(
        "INTAKE_CAMERA",
        new Transform3d()
    );

    public static double GROUND_PARALLEL_THRESHOLD = 1e-9;
}
