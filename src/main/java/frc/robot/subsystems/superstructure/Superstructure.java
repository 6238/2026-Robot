package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
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
  public IntakeRoller intakeRoller;

  public SwerveDriveSimulation swerveDriveSimulation;

  public enum WantedState {
    IDLE,
    INTAKING,
    SHOOTING,
    PASSING
  }

  public enum CurrentState {
    IDLE,
    INTAKING,
    SPINNING_UP,
    REVERSING,
    SHOOTING,
    PASSING
  }

  public WantedState wantedSuperState = WantedState.IDLE;
  public CurrentState currentSuperState = CurrentState.IDLE;

  private ShotSetpoint shotSetpoint = new ShotSetpoint();

  private double shotSimulationTime = 0.0;
  private final Timer reversingTimer = new Timer();
  private final Timer noShotTimer = new Timer();
  // Prevents jam-recovery from re-triggering for 1.5 s after it last fired
  private final Timer noShotCooldownTimer = new Timer();
  private boolean crawlUpScheduled = false;

  // Inlined crawlUp state
  private double crawlBackoffTimer = 0.0;
  private boolean crawlIsBackingOff = false;
  private boolean crawlBackoffTriggered = false;

  // Inlined top-indexer oscillation state
  private final Timer topIndexerOscillateTimer = new Timer();
  private boolean topIndexerHighPhase = false;

  public boolean indxererMode = false;

  public Superstructure(
      Drive drive,
      Shooter shooter,
      Hopper hopper,
      IntakePivot intake,
      IntakeRoller intakeRoller,
      SwerveDriveSimulation swerveDriveSimulation) {
    this.drive = drive;
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.intakeRoller = intakeRoller;
    this.swerveDriveSimulation = swerveDriveSimulation;
    noShotCooldownTimer.start(); // already-elapsed at match start
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
        if (currentSuperState != CurrentState.IDLE) {
          currentSuperState = CurrentState.IDLE;
          crawlUpScheduled = false;
          crawlIsBackingOff = false;
          hopper.setIndexerVolts(0);
          hopper.setTopIndexerVolts(0);
          shooter.setFlywheelVoltage(Volts.of(0));
          shooter.setFeederVoltage(Volts.of(0));
          intakeRoller.stop();
          intake.io.setIntakeArmVoltage(Volts.of(0));
        }
        break;
      case INTAKING:
        if (currentSuperState != CurrentState.INTAKING) currentSuperState = CurrentState.INTAKING;
        break;
      case SHOOTING:
        if (currentSuperState != CurrentState.SHOOTING
            && currentSuperState != CurrentState.REVERSING
            && currentSuperState != CurrentState.SPINNING_UP)
          currentSuperState = CurrentState.SPINNING_UP;
        break;
      case PASSING:
        if (currentSuperState != CurrentState.PASSING
            && currentSuperState != CurrentState.SPINNING_UP)
          currentSuperState = CurrentState.SPINNING_UP;
        break;
      default:
        if (currentSuperState != CurrentState.IDLE) {
          currentSuperState = CurrentState.IDLE;
          crawlUpScheduled = false;
          crawlIsBackingOff = false;
          hopper.setIndexerVolts(0);
          hopper.setTopIndexerVolts(0);
          shooter.setFlywheelVoltage(Volts.of(0));
          shooter.setFeederVoltage(Volts.of(0));
          intakeRoller.stop();
          intake.io.setIntakeArmVoltage(Volts.of(0));
        }
        break;
    }
  }

  public void applyStates() {
    switch (currentSuperState) {
      case IDLE:
        break;
      case INTAKING:
        intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        intakeRoller.spin();
        break;
      case SPINNING_UP:
        intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        intakeRoller.spin();
        hopper.setIndexerVolts(0);
        shooter.setFeederSpeed(RotationsPerSecond.of(ShooterConstants.FEEDER_SPEED.get()));
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        if (!readyToShoot()) break;
        if (wantedSuperState == WantedState.SHOOTING) {
          currentSuperState = CurrentState.REVERSING;
          reversingTimer.restart();
        } else if (wantedSuperState == WantedState.PASSING) {
          currentSuperState = CurrentState.PASSING;
        }
        break;
      case REVERSING:
        // Briefly reverse feeder and indexer to prevent jams before the shot
        intakeRoller.spin();
        hopper.setIndexerVolts(-HopperConstants.INDEXER_VOLTAGE.get());
        hopper.setTopIndexerVolts(-HopperConstants.TOP_INDEXER_VOLTAGE.get());
        shooter.setFeederVoltage(Volts.of(-ShooterConstants.FEEDER_REVERSE_VOLTAGE.get()));
        if (reversingTimer.hasElapsed(0.10)) {
          reversingTimer.stop();
          currentSuperState = CurrentState.SHOOTING;
          noShotTimer.restart();
          // Start indexer and reset oscillation / crawl state for SHOOTING
          // topIndexerOscillateTimer.restart();
          // topIndexerHighPhase = false;
          crawlIsBackingOff = false;
          crawlBackoffTimer = 0.0;
          crawlBackoffTriggered = false;
        }
        break;
      case PASSING:
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        shooter.setFeederSpeed(RotationsPerSecond.of(ShooterConstants.FEEDER_SPEED.get()));
        simulateShot();
        break;
      case SHOOTING:
        intakeRoller.spin();
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        shooter.setFeederSpeed(RotationsPerSecond.of(ShooterConstants.FEEDER_SPEED.get()));
        hopper.setIndexerVolts(HopperConstants.INDEXER_VOLTAGE.get());
        hopper.setTopIndexerVolts(HopperConstants.TOP_INDEXER_VOLTAGE.get());

        // Top-indexer oscillation (inlined from hopper.oscillateTopIndexer())
        // if (!topIndexerHighPhase && topIndexerOscillateTimer.hasElapsed(0.2)) {
        //   topIndexerHighPhase = true;
        //   topIndexerOscillateTimer.restart();
        //   hopper.setTopIndexerVolts(-HopperConstants.TOP_INDEXER_VOLTAGE.get());
        // } else if (topIndexerHighPhase && topIndexerOscillateTimer.hasElapsed(0.7)) {
        //   topIndexerHighPhase = false;
        //   topIndexerOscillateTimer.restart();
        //   hopper.setTopIndexerVolts(0);
        // }

        if (readyToShoot() && !crawlUpScheduled) {
          crawlUpScheduled = true;
          crawlIsBackingOff = false;
          crawlBackoffTimer = 0.0;
          crawlBackoffTriggered = false;
        }
        if (crawlUpScheduled) applyCrawlUp();

        // Reset no-shot timer whenever a ball exits (voltage spikes down: bang-through → PID/FF)
        if (shooter.ballExitedFlywheel()) noShotTimer.restart();

        if (!checkHubTolerance()) {
          currentSuperState = CurrentState.SPINNING_UP;
        }

        // If no ball has been shot in 1.0 s and cooldown has passed, drop intake and restart
        if (noShotTimer.hasElapsed(1.0) && noShotCooldownTimer.hasElapsed(1.5)) {
          noShotTimer.restart();
          noShotCooldownTimer.restart();
          crawlUpScheduled = false;
          crawlIsBackingOff = false;
          crawlBackoffTimer = 0.0;
          intake.io.setIntakeArmVoltage(Volts.of(0));
          intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        }

        simulateShot();
        break;
      default:
        break;
    }
  }

  private void applyCrawlUp() {
    double positionDeg = intake.inputs.intakeArmPosition.in(Degrees);
    double maxDeg =
        IntakeConstants.INTAKE_UP_VALUE.get() + IntakeConstants.CRAWL_MAX_OFFSET_DEGREES;

    if (positionDeg >= maxDeg) {
      intake.io.setIntakeArmVoltage(Volts.of(0));
      return;
    }

    double cosScale = Math.cos(Math.toRadians(positionDeg));
    double scaledVoltage =
        Math.max(
            IntakeConstants.CRAWL_UP_VOLTAGE_VOLTS.get() * cosScale,
            IntakeConstants.CRAWL_UP_VOLTAGE_MIN_VOLTS.get());
    double scaledCurrentThreshold =
        Math.max(
            IntakeConstants.CRAWL_CURRENT_THRESHOLD_AMPS.get() * cosScale,
            IntakeConstants.CRAWL_CURRENT_THRESHOLD_MIN_AMPS.get());

    if (crawlIsBackingOff) {
      crawlBackoffTimer += 0.02;
      intake.io.setIntakeArmVoltage(Volts.of(-IntakeConstants.CRAWL_BACKOFF_VOLTAGE_VOLTS.get()));
      if (crawlBackoffTimer >= IntakeConstants.CRAWL_BACKOFF_DURATION_SECONDS.get()
          || positionDeg <= IntakeConstants.INTAKE_DOWN_VALUE.get()) {
        crawlIsBackingOff = false;
        crawlBackoffTimer = 0.0;
      }
    } else {
      boolean aboveDeadzone = positionDeg > IntakeConstants.INTAKE_DOWN_VALUE.get() + 10.0;
      if (aboveDeadzone
          && intake.inputs.intakeArmSupplyCurrent.in(Amps) >= scaledCurrentThreshold) {
        crawlIsBackingOff = true;
        crawlBackoffTriggered = true;
        crawlBackoffTimer = 0.0;
      } else {
        crawlBackoffTriggered = false;
        intake.io.setIntakeArmVoltage(Volts.of(scaledVoltage));
      }
    }

    Logger.recordOutput("IntakePivot/CrawlIsBackingOff", crawlIsBackingOff);
    Logger.recordOutput("IntakePivot/CrawlBackoffTriggered", crawlBackoffTriggered);
    Logger.recordOutput("IntakePivot/CrawlScaledVoltage", scaledVoltage);
    Logger.recordOutput("IntakePivot/CrawlScaledCurrentThreshold", scaledCurrentThreshold);
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
    boolean shooterSpeedSetpoint = shooter.flywheelUpToSpeed(); // && shooter.feederUpToSpeed();
    boolean hubSetpoint = checkHubTolerance();

    Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterSpeedSetpoint);
    Logger.recordOutput("Superstructure/HubRotationSetpoint", hubSetpoint);
    Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose);
    Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose());
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

  public Command wantIntaking() {
    return startEnd(
        () -> setWantedSuperState(WantedState.INTAKING),
        () -> setWantedSuperState(WantedState.IDLE));
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
