package frc.robot.subsystems.objectdetection;

import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.objectdetection.ObjectDetectionIO.FuelObservation;

public class ObjectDetection extends SubsystemBase {
    private ObjectDetectionIO io;
    public ObjectDetectionInputsAutoLogged inputs;

    private ArrayList<FuelTransformedObservation> currentTransformedObservations = new ArrayList<>();

    public ObjectDetection(ObjectDetectionIO io) {
        this.io = io;
        this.inputs = new ObjectDetectionInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectDetection", inputs);

        for (FuelObservation fuelObservation : inputs.targets) {
            Optional<FuelTransformedObservation> transformedObservation = toGroundIntersection(fuelObservation, robotToCamera0);
            if (transformedObservation.isPresent()) {
                currentTransformedObservations.add(transformedObservation.get());
            }
        }

        Logger.recordOutput(
            "ObjectDetection/transformedTargetTranslations", 
            currentTransformedObservations.stream().map((transformedObservation) -> transformedObservation.location).toArray(Translation2d[]::new)
        );
        Logger.recordOutput(
            "ObjectDetection/transformedTargets", 
            currentTransformedObservations.toArray(new FuelTransformedObservation[0])
        );
    }

    public FuelTransformedObservation[] getFuelTransformedObservations() {
        return currentTransformedObservations.toArray(new FuelTransformedObservation[0]);
    }

    private Optional<FuelTransformedObservation> toGroundIntersection(
      FuelObservation observation, Transform3d robotToCamera) {

        Translation3d camPosRobot = robotToCamera.getTranslation();
        Rotation3d camRotationToRay = new Rotation3d(
            0.0,
            observation.pitch().getRadians(),
            observation.yaw().getRadians()
        );

        Translation3d rayDirCamera = new Translation3d(1.0, 0.0, 0.0).rotateBy(camRotationToRay);
        Translation3d rayDirRobot = rayDirCamera.rotateBy(robotToCamera.getRotation());

        double deltaZDir = rayDirRobot.getZ();
        if (Math.abs(deltaZDir) < ObjectDetectionConstants.GROUND_PARALLEL_THRESHOLD) return Optional.empty();

        double intersectionTime = -camPosRobot.getZ() / deltaZDir;
        if (intersectionTime <= 0.0) return Optional.empty();

        Translation3d hit = camPosRobot.plus(rayDirRobot.times(intersectionTime));
        Translation2d hit2d = new Translation2d(hit.getX(), hit.getY());

        return Optional.of(new ObjectDetection.FuelTransformedObservation(hit2d, observation.area(), observation.timestamp()));
    }

    public static record FuelTransformedObservation(Translation2d location, double area, double timestamp) {};
}
