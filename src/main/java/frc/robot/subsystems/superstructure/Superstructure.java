package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.IntakePivot;
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
  public IntakePivot intake;

  public SwerveDriveSimulation swerveDriveSimulation;

  public enum WantedState {
    IDLE,
    SHOOTING,
    PASSING
  }

  public enum CurrentState {
    IDLE,
    SPINNING_UP,
    SHOOTING,
    PASSING
  }

  public WantedState wantedSuperState = WantedState.IDLE;
  public CurrentState currentSuperState = CurrentState.IDLE;

  private ShotSetpoint shotSetpoint = new ShotSetpoint();

  private double shotSimulationTime = 0.0;

  public boolean indxererMode = false;

  public Superstructure(
      Drive drive,
      Shooter shooter,
      Hopper hopper,
      IntakePivot intake,
      SwerveDriveSimulation swerveDriveSimulation) {
    this.drive = drive;
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
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
      case PASSING:
        if (currentSuperState != CurrentState.PASSING) currentSuperState = CurrentState.SPINNING_UP;
        break;
      default:
        currentSuperState = CurrentState.IDLE;
        break;
    }
  }

  public void applyStates() {
    switch (currentSuperState) {
      case IDLE:
        CommandScheduler.getInstance()
            .schedule(
                hopper.stopFullIndexer(),
                shooter.setFlywheelVoltage(() -> Volts.of(0)),
                shooter.setFeederVoltage(() -> Volts.of(0)));
        break;
      case SPINNING_UP:
        CommandScheduler.getInstance()
            .schedule(
                hopper.stopIndexer(),
                shooter.setFeederVoltage(() -> Volts.of(ShooterConstants.FEEDER_VOLTAGE.get())),
                shooter.setFlywheelRPM(() -> shotSetpoint.flywheelSpeed));

        // if (!readyToSpinTopIndexer()) {
        //   CommandScheduler.getInstance().schedule(hopper.stopTopIndexer());
        //   break;
        // }
        // CommandScheduler.getInstance().schedule(hopper.spinTopIndexer());

        if (!readyToShoot()) {
          break;
        }
        if (wantedSuperState == WantedState.SHOOTING) {
          currentSuperState = CurrentState.SHOOTING;
          CommandScheduler.getInstance()
              .schedule(Commands.sequence(hopper.spinIndexer(), hopper.oscillateTopIndexer()));
        } else if (wantedSuperState == WantedState.PASSING) {
          currentSuperState = CurrentState.PASSING;
        }
        break;
      case PASSING:
      case SHOOTING:
        CommandScheduler.getInstance()
            .schedule(
                shooter.setFlywheelRPM(() -> shotSetpoint.flywheelSpeed),
                shooter.setFeederVoltage(() -> Volts.of(ShooterConstants.FEEDER_VOLTAGE.get())));
        if (!checkHubTolerance()) {
          currentSuperState = CurrentState.SPINNING_UP;
          break;
        }

        simulateShot();
        // if (indxererMode) {
        //   CommandScheduler.getInstance().schedule(hopper.spinFullIndexer(3, 3));
        //   break;
        // }
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
                    shotSetpoint.hoodAngle)
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

  public boolean readyToSpinTopIndexer() {
    return shooter.flywheelUpToSpeed(HopperConstants.HOPPER_TOLERANCE_BEFORE_SHOT);
  }

  public boolean isPrettyMuchCloseToTargetButNotQuite() {
    boolean shooterSpeedSetpoint =
        shooter.flywheelUpToSpeed(ShooterConstants.BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT);
    boolean hubSetpoint = checkHubTolerance();

    Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterSpeedSetpoint);
    Logger.recordOutput("Superstructure/HubRotationSetpoint", hubSetpoint);
    Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose.getRotation());
    Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose().getRotation());
    Logger.recordOutput(
        "Superstructure/HubRotationError",
        Math.abs(
            drive
                .getPose()
                .getRotation()
                .minus(shotSetpoint.robotPose.getRotation())
                .getDegrees()));

    return shooterSpeedSetpoint && hubSetpoint;
  }

  private double getDynamicHubToleranceDegrees() {
    double distance = drive.getPose().getTranslation().getDistance(shotSetpoint.target);
    boolean nearHub = distance < ShooterConstants.HUB_NEAR_DISTANCE_METERS;
    var speeds = drive.getChassisSpeeds();
    double robotSpeedMps = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    boolean highSpeed = robotSpeedMps > ShooterConstants.HUB_HIGH_ROBOT_SPEED_MPS;
    if (nearHub || highSpeed) {
      return ShooterConstants.HUB_ROTATION_TOLERANCE_TIGHT.in(Degrees);
    }
    return ShooterConstants.HUB_ROTATION_TOLERANCE.in(Degrees);
  }

  public boolean checkHubTolerance() {
    double rotationError =
        Math.abs(drive.getRotation().minus(shotSetpoint.robotPose.getRotation()).getDegrees());
    boolean hubRotationWithinTolerance = rotationError < getDynamicHubToleranceDegrees();
    return hubRotationWithinTolerance;
  }

  public boolean readyToShoot() {
    boolean shooterSpeedSetpoint = shooter.flywheelUpToSpeed();
    boolean hubSetpoint = checkHubTolerance();

    Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterSpeedSetpoint);
    Logger.recordOutput("Superstructure/HubRotationSetpoint", hubSetpoint);
    Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose.getRotation());
    Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose().getRotation());
    Logger.recordOutput("Superstructure/HubRotationToleranceDeg", getDynamicHubToleranceDegrees());
    Logger.recordOutput(
        "Superstructure/HubRotationError",
        Math.abs(
            drive
                .getPose()
                .getRotation()
                .minus(shotSetpoint.robotPose.getRotation())
                .getDegrees()));

    return shooterSpeedSetpoint && hubSetpoint;
  }

  public ShotSetpoint getShotSetpoint() {
    return shotSetpoint;
  }

  @Override
  public void periodic() {
    if (wantedSuperState == WantedState.SHOOTING)
      shotSetpoint = ShotPlanner.createShotSetpoint(drive.getPose(), drive.getChassisSpeeds());
    else if (wantedSuperState == WantedState.PASSING) {
      shotSetpoint =
          ShotPlanner.createPassSetpoint(
              drive.getPose().getY() > Constants.LEFT_RIGHT_SPLIT
                  ? Constants.LEFT_TARGET_PASS_POSE2D.get()
                  : Constants.RIGHT_TARGET_PASS_POSE2D.get(),
              drive.getPose(),
              drive.getChassisSpeeds());
    }

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
