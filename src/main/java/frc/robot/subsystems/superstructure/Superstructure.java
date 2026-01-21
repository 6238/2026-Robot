package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.superstructure.ShotPlanner.ShotSetpoint;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  public Drive drive;
  public Shooter shooter;
  public Hopper hopper;

  public SwerveDriveSimulation swerveDriveSimulation;

  public enum WantedState {
    IDLE,
    SHOOTING
  }

  public enum CurrentState {
    IDLE,
    SPINNING_UP,
    SHOOTING
  }

  public WantedState wantedSuperState = WantedState.IDLE;
  public CurrentState currentSuperState = CurrentState.IDLE;

  private ShotSetpoint shotSetpoint = new ShotSetpoint();

  private double shotSimulationTime = 0.0;

  public Superstructure(
      Drive drive, Shooter shooter, Hopper hopper, SwerveDriveSimulation swerveDriveSimulation) {
    this.drive = drive;
    this.shooter = shooter;
    this.hopper = hopper;
    this.swerveDriveSimulation = swerveDriveSimulation;
  }

  public void setWantedSuperState(WantedState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
    handleWantedState();
  }

  public Command setWantedSuperStateCommand(Supplier<WantedState> wantedSuperState) {
    return runOnce(
        () -> {
          this.wantedSuperState = wantedSuperState.get();
          handleWantedState();
        });
  }

  public void handleWantedState() {
    switch (wantedSuperState) {
      case IDLE:
        currentSuperState = CurrentState.IDLE;
        break;
      case SHOOTING:
        if (currentSuperState != CurrentState.SHOOTING)
          currentSuperState = CurrentState.SPINNING_UP;
        break;
      default:
        currentSuperState = CurrentState.IDLE;
        break;
    }
  }

  public void applyStates() {
    switch (currentSuperState) {
      case IDLE:
        CommandScheduler.getInstance().schedule(hopper.stopIndexer());
        CommandScheduler.getInstance()
            .schedule(shooter.setFlywheelRPM(() -> RotationsPerSecond.of(0)));
        CommandScheduler.getInstance().schedule(shooter.setFeederVoltage(() -> Volts.of(0)));
        break;
      case SPINNING_UP:
        CommandScheduler.getInstance().schedule(hopper.stopIndexer());
        CommandScheduler.getInstance().schedule(shooter.setFeederVoltage(() -> Volts.of(0)));
        CommandScheduler.getInstance()
            .schedule(shooter.setFlywheelRPM(() -> shotSetpoint.flywheelSpeed));

        if (readyToShoot()) {
          currentSuperState = CurrentState.SHOOTING;
        }
        break;
      case SHOOTING:
        CommandScheduler.getInstance()
            .schedule(shooter.setFlywheelRPM(() -> shotSetpoint.flywheelSpeed));
        if (!readyToShoot()) {
          currentSuperState = CurrentState.SPINNING_UP;
          break;
        }

        simulateShot();
        CommandScheduler.getInstance().schedule(hopper.spinIndexer());
        CommandScheduler.getInstance()
            .schedule(
                shooter.setFeederVoltage(() -> Volts.of(ShooterConstants.FEEDER_VOLTAGE.get())));
        break;
      default:
        break;
    }
  }

  public void simulateShot() {
    if (Constants.currentMode != Mode.SIM) {
      return;
    }

    if (shotSimulationTime < 1 / HopperConstants.simulatedHopperThroughput) {
      return;
    }
    shotSimulationTime = 0.0;

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                    swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                    new Translation2d(Units.inchesToMeters(12), 0),
                    swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation(),
                    Inches.of(21),
                    MetersPerSecond.of(
                        shotSetpoint.flywheelSpeed.in(RotationsPerSecond)
                            * 2
                            * 0.0508
                            * Math.PI
                            / 1.9),
                    ShooterConstants.FIXED_HOOD_ANGLE_DEGREES)
                .withProjectileTrajectoryDisplayCallBack(
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/NoteProjectileSuccessfulShot",
                            pose3ds.toArray(Pose3d[]::new)),
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/NoteProjectileUnsuccessfulShot",
                            pose3ds.toArray(Pose3d[]::new)))
                .withTargetPosition(
                    () ->
                        FieldMirroringUtils.toCurrentAllianceTranslation(
                            new Translation3d(1.52, 4.11, 1.83)))
                .withTargetTolerance(new Translation3d(0.52, 0.52, 0.25))
                .withHitTargetCallBack(() -> System.out.println("Scored fuel in Hub, +1 point!")));
  }

  public boolean checkHubTolerance() {
    double rotationError =
        Math.abs(
            drive.getPose().getRotation().minus(shotSetpoint.robotPose.getRotation()).getDegrees());
    boolean hubRotationWithinTolerance =
        Math.abs(rotationError) < ShooterConstants.HUB_ROTATION_TOLERANCE.in(Degrees);
    return hubRotationWithinTolerance;
  }

  public boolean readyToShoot() {
    boolean shooterSpeedSetpoint = shooter.flywheelUpToSpeed();
    boolean hubSetpoint = checkHubTolerance();

    Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterSpeedSetpoint);
    Logger.recordOutput("Superstructure/HubRotationSetpoint", hubSetpoint);

    return shooterSpeedSetpoint && hubSetpoint;
  }

  public ShotSetpoint getShotSetpoint() {
    return shotSetpoint;
  }

  @Override
  public void periodic() {
    shotSetpoint = ShotPlanner.createShotSetpoint(drive.getPose(), drive.getChassisSpeeds());

    handleWantedState();
    applyStates();

    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
  }

  @Override
  public void simulationPeriodic() {
    shotSimulationTime += 0.02;
  }
}
