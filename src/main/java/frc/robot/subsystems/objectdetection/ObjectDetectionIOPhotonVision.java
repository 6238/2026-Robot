package frc.robot.subsystems.objectdetection;


import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;

public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
    public PhotonCamera camera;
    public ObjectDetectionCameraSettings cameraSettings;

    public ObjectDetectionIOPhotonVision(ObjectDetectionCameraSettings cameraSettings) {
        this.camera = new PhotonCamera(cameraSettings.cameraName);
        this.cameraSettings = cameraSettings;
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        inputs.connected = camera.isConnected();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        List<FuelObservation> fuelObservations = new ArrayList<>();

        for (PhotonPipelineResult photonPipelineResult : results) {
            List<PhotonTrackedTarget> targets = photonPipelineResult.getTargets();
            double captureTime = photonPipelineResult.getTimestampSeconds();

            for (PhotonTrackedTarget target : targets) {
                fuelObservations.add(new FuelObservation(
                    new Rotation2d(target.getPitch()),
                    new Rotation2d(target.getYaw()),
                    target.getArea(),
                    captureTime
                ));
            }
        }
    }

    @Override
    public ObjectDetectionCameraSettings getCameraSettings() {
        return cameraSettings;
    }
}
