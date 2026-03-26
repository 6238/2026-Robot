package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.RobotTunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class MapleSimSwerve {

  @SuppressWarnings("unchecked")
  public static SwerveDriveSimulation createSimulationDrive(RobotTunerConstants constants) {
    SwerveDriveSimulation sim =
        new SwerveDriveSimulation(
            new DriveTrainSimulationConfig(
                Pounds.of(90),
                Inches.of(27),
                Inches.of(27),
                Meters.of(constants.FrontLeft.LocationX - constants.BackLeft.LocationX),
                Meters.of(constants.FrontLeft.LocationY - constants.FrontRight.LocationY),
                COTS.ofPigeon2(),
                COTS.ofSwerveX2(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), 1.4, 2, 11)),
            new Pose2d(0, 0, new Rotation2d(0)));
    SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    return sim;
  }

  @SuppressWarnings("unchecked")
  public static SwerveDriveSimulation createSimulationDrive(
      RobotTunerConstants constants, Pose2d pose) {
    SwerveDriveSimulation sim =
        new SwerveDriveSimulation(
            new DriveTrainSimulationConfig(
                Pounds.of(90),
                Inches.of(29.5 + 4),
                Inches.of(29.5 + 4),
                Meters.of(constants.FrontLeft.LocationX - constants.BackLeft.LocationX),
                Meters.of(constants.FrontLeft.LocationY - constants.FrontRight.LocationY),
                COTS.ofPigeon2(),
                COTS.ofSwerveX2(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), 1.4, 2, 11)),
            pose);
    SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    return sim;
  }
}
