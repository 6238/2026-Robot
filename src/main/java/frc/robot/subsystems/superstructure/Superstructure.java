package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private Drive drive;
  private Shooter shooter;
  private Hopper hopper;
  private IntakePivot intake;
  private IntakeRoller intakeRoller;

  public SwerveDriveSimulation swerveDriveSimulation;

  public enum WantedState {
    IDLE,
    INTAKING,
    SHOOTING,
    PASSING,
    SHOOT_INTAKE,
    PASS_INTAKE,
    PIT_SHOOT
  }

  public enum CurrentState {
    IDLE,
    INTAKING,
    SPINNING_UP,
    SHOOTING,
    PASSING,
    PIT_SHOOTING
  }

  public WantedState wantedSuperState = WantedState.IDLE;
  public CurrentState currentSuperState = CurrentState.IDLE;

  private ShotSetpoint shotSetpoint = new ShotSetpoint();

  private double shotSimulationTime = 0.0;
  private final Timer noShotTimer = new Timer();
  // Prevents jam-recovery from re-triggering for 1.5 s after it last fired
  private final Timer noShotCooldownTimer = new Timer();
  // Holds flywheel at shot speed for 0.5 s after returning to IDLE
  private final Timer postShotHoldTimer = new Timer();
  private static final double POST_SHOT_HOLD_SECONDS = 0.5;
  private boolean crawlUpScheduled = false;
  // Consecutive loops where both flywheel and hub are within entry tolerance; must reach 2 before
  // transitioning SPINNING_UP -> SHOOTING/PASSING (prevents heading-overshoot false-positives).
  private int hubInToleranceLoops = 0;

  // Pivot oscillation state (during shooting/passing)
  private double oscTimer = 0.0;

  // Jam detection: feeder high current + low velocity → reverse for JAM_REVERSE_DURATION_SECONDS
  private final edu.wpi.first.math.filter.Debouncer jamDetectDebouncer =
      new edu.wpi.first.math.filter.Debouncer(
          0.1, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);
  private final Timer jamReverseTimer = new Timer();
  private final Timer jamCooldownTimer = new Timer();
  private boolean jamReverseActive = false;

  // Beam-break intake sequencing: go down after each shot, wait for next ball to resume oscillating
  private boolean intakeWaitingForNextBall = false;
  private boolean prevBeamBreak = false;
  private final Timer intakeDownTimer = new Timer();

  // 254 push indexing sub-state
  private enum Push254Phase {
    PUSHING,
    OSCILLATING,
    RETRACTING
  }

  private Push254Phase push254Phase = Push254Phase.PUSHING;
  private final Timer push254Timer = new Timer();
  private final edu.wpi.first.math.filter.Debouncer push254JamDebouncer =
      new edu.wpi.first.math.filter.Debouncer(
          0.1, edu.wpi.first.math.filter.Debouncer.DebounceType.kRising);

  /**
   * Called in simulateShot() to consume one fuel piece before launching a projectile. Set by
   * RobotContainer in SIM mode to {@code fuelIntake::obtainGamePieceFromIntake}. Returns true if a
   * piece was available and consumed; false skips the shot.
   */
  public BooleanSupplier consumeFuelForShot = () -> true;

  /**
   * Returns true when the flywheel should pre-spin in IDLE due to an active or imminent hub shift.
   * Set by RobotContainer based on match timing and game data.
   */
  public BooleanSupplier hubSpinupActive = () -> false;

  private boolean wasHubSpinupActive = false;

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
    postShotHoldTimer.start(); // already-elapsed at match start
    jamCooldownTimer.start(); // already-elapsed at match start
  }

  public void setWantedSuperState(WantedState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
  }

  public Command setWantedSuperStateCommand(Supplier<WantedState> wantedSuperState) {
    return runOnce(() -> this.wantedSuperState = wantedSuperState.get());
  }

  public void handleWantedState() {
    switch (wantedSuperState) {
      case IDLE:
        if (currentSuperState != CurrentState.IDLE) {
          boolean wasShootingOrPassing =
              currentSuperState == CurrentState.SHOOTING
                  || currentSuperState == CurrentState.PASSING
                  || currentSuperState == CurrentState.SPINNING_UP;
          if (wasShootingOrPassing) postShotHoldTimer.restart();
          currentSuperState = CurrentState.IDLE;
          crawlUpScheduled = false;
          intakeWaitingForNextBall = false;
          prevBeamBreak = false;
          jamReverseActive = false;
          jamDetectDebouncer.calculate(false);
          hopper.setIndexerSpeed(RotationsPerSecond.of(0));
          hopper.setTopIndexerSpeed(RotationsPerSecond.of(0));
          shooter.setFeederVoltage(Volts.of(0));
          intakeRoller.stop();
          intake.io.setIntakeArmVoltage(Volts.of(0));
          if (!wasShootingOrPassing) shooter.setFlywheelVoltage(Volts.of(0));
        }
        break;
      case INTAKING:
        if (currentSuperState != CurrentState.INTAKING) currentSuperState = CurrentState.INTAKING;
        break;
      case SHOOTING:
        if (currentSuperState != CurrentState.SHOOTING
            && currentSuperState != CurrentState.SPINNING_UP) {
          currentSuperState = CurrentState.SPINNING_UP;
          hubInToleranceLoops = 0;
        }
        break;
      case PASSING:
        if (currentSuperState != CurrentState.PASSING
            && currentSuperState != CurrentState.SPINNING_UP) {
          currentSuperState = CurrentState.SPINNING_UP;
          hubInToleranceLoops = 0;
        }
        break;
      case SHOOT_INTAKE:
        if (currentSuperState != CurrentState.SHOOTING
            && currentSuperState != CurrentState.SPINNING_UP) {
          currentSuperState = CurrentState.SPINNING_UP;
          hubInToleranceLoops = 0;
        }
        break;
      case PASS_INTAKE:
        if (currentSuperState != CurrentState.PASSING
            && currentSuperState != CurrentState.SPINNING_UP) {
          currentSuperState = CurrentState.SPINNING_UP;
          hubInToleranceLoops = 0;
        }
        break;
      case PIT_SHOOT:
        if (currentSuperState != CurrentState.PIT_SHOOTING) {
          currentSuperState = CurrentState.PIT_SHOOTING;
          crawlUpScheduled = false;
          oscTimer = 0.0;
        }
        break;
      default:
        if (currentSuperState != CurrentState.IDLE) {
          currentSuperState = CurrentState.IDLE;
          crawlUpScheduled = false;
          hopper.setIndexerSpeed(RotationsPerSecond.of(0));
          hopper.setTopIndexerSpeed(RotationsPerSecond.of(0));
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
        if (!postShotHoldTimer.hasElapsed(POST_SHOT_HOLD_SECONDS)) {
          shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
          break;
        }
        boolean shouldPreSpin =
            (ShooterConstants.SPINUP_WHEN_IDLE && DriverStation.isEnabled())
                || (ShooterConstants.SPINUP_WHEN_HUB_ACTIVE && hubSpinupActive.getAsBoolean());
        if (shouldPreSpin) {
          shooter.setFlywheelRPM(
              RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get()));
          wasHubSpinupActive = true;
        } else {
          if (wasHubSpinupActive) wasHubSpinupActive = false;
          shooter.setFlywheelVoltage(Volts.of(0));
        }
        break;
      case INTAKING:
        intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        intakeRoller.spin();
        hopper.setIndexerSpeed(RotationsPerSecond.of(0));
        hopper.setTopIndexerSpeed(RotationsPerSecond.of(0));
        shooter.setFeederVoltage(Volts.of(0));
        // if (ShooterConstants.SPINUP_WHEN_HUB_ACTIVE && hubSpinupActive.getAsBoolean()) {
        shooter.setFlywheelRPM(RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get()));
        hopper.spinFullIndexer(RotationsPerSecond.of(-20), RotationsPerSecond.of(-20));
        // wasHubSpinupActive = true;
        // } else if (wasHubSpinupActive) {
        //   shooter.setFlywheelVoltage(Volts.of(0));
        //   hopper.stopFullIndexer();
        //   wasHubSpinupActive = false;
        // }
        break;
      case SPINNING_UP:
        if (wantedSuperState == WantedState.SHOOT_INTAKE
            || wantedSuperState == WantedState.PASS_INTAKE) {
          intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
          intakeRoller.spin();
        } else {
          intake.setAngle(Degrees.of(IntakeConstants.SPINNING_UP_INTAKE_ANGLE.get()));
        }
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        shooter.setFeederVoltage(Volts.of(-ShooterConstants.FEEDER_REVERSE_VOLTAGE.get()));
        hopper.setIndexerSpeed(RotationsPerSecond.of(-15));
        hopper.setTopIndexerSpeed(RotationsPerSecond.of(-15));
        boolean isPassing =
            wantedSuperState == WantedState.PASSING || wantedSuperState == WantedState.PASS_INTAKE;
        boolean flywheelReady =
            isPassing
                ? shooter.getCurrentFlywheelSpeed().in(RotationsPerSecond)
                    >= ShooterConstants.PASSING_FLYWHEEL_MIN_RPS
                : shooter.flywheelUpToSpeed(ShooterConstants.FLYWHEEL_SPINUP_TRANSITION_TOLERANCE);
        if (flywheelReady && checkHubTolerance()) {
          hubInToleranceLoops++;
        } else {
          hubInToleranceLoops = 0;
        }
        if (hubInToleranceLoops < 2) break;
        if (wantedSuperState == WantedState.SHOOTING
            || wantedSuperState == WantedState.SHOOT_INTAKE) {
          currentSuperState = CurrentState.SHOOTING;
          noShotTimer.restart();
          crawlUpScheduled = false;
          oscTimer = 0.0;
        } else if (wantedSuperState == WantedState.PASSING
            || wantedSuperState == WantedState.PASS_INTAKE) {
          currentSuperState = CurrentState.PASSING;
          crawlUpScheduled = false;
          oscTimer = 0.0;
        }
        break;
      case PASSING:
        intakeRoller.spin();
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        shooter.setFeederSpeed(shotSetpoint.feederSpeed);
        var passSpeeds = drive.getChassisSpeeds();
        boolean passingOnMove =
            Math.hypot(passSpeeds.vxMetersPerSecond, passSpeeds.vyMetersPerSecond)
                > ShooterConstants.HUB_HIGH_ROBOT_SPEED_MPS;
        double passIndexerScale = passingOnMove ? 0.5 : 1.0;
        hopper.setIndexerSpeed(
            RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get() * passIndexerScale));
        hopper.setTopIndexerSpeed(
            RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get() * passIndexerScale));
        applyFeedingLogic();
        if (getHubRotationError() > ShooterConstants.HUB_PASSING_DROPBACK_TOLERANCE.in(Degrees)) {
          currentSuperState = CurrentState.SPINNING_UP;
          hubInToleranceLoops = 0;
        }
        break;
      case PIT_SHOOTING:
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        shooter.setFeederSpeed(shotSetpoint.feederSpeed);
        hopper.setIndexerSpeed(RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get()));
        hopper.setTopIndexerSpeed(RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get()));
        applyFeedingLogic();
        break;
      case SHOOTING:
        intakeRoller.spin();
        shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
        shooter.setFeederSpeed(shotSetpoint.feederSpeed);
        boolean tooCloseShooting = isTooCloseToHub();
        if (!Constants.MINIMAL_LOGGING)
          Logger.recordOutput("Superstructure/TooCloseToShoot", tooCloseShooting);
        var shootSpeeds = drive.getChassisSpeeds();
        boolean shootingOnMove =
            Math.hypot(shootSpeeds.vxMetersPerSecond, shootSpeeds.vyMetersPerSecond)
                > ShooterConstants.HUB_HIGH_ROBOT_SPEED_MPS;
        double indexerScale = shootingOnMove ? 0.5 : 1.0;
        hopper.setTopIndexerSpeed(
            RotationsPerSecond.of(
                tooCloseShooting ? 0 : HopperConstants.INDEXER_SPEED.get() * indexerScale));
        hopper.setIndexerSpeed(
            RotationsPerSecond.of(
                tooCloseShooting ? 0 : HopperConstants.INDEXER_SPEED.get() * indexerScale));

        boolean shotFired = applyFeedingLogic();
        if (shotFired) noShotTimer.restart();

        if (getHubRotationError() > ShooterConstants.HUB_DROPBACK_TOLERANCE.in(Degrees)) {
          currentSuperState = CurrentState.SPINNING_UP;
          hubInToleranceLoops = 0;
        }

        // If no ball has been shot in 1.0 s and cooldown has passed, drop intake and restart
        if (noShotTimer.hasElapsed(0.3) && noShotCooldownTimer.hasElapsed(0.75)) {
          noShotTimer.restart();
          noShotCooldownTimer.restart();
          crawlUpScheduled = false;
          oscTimer = 0.0;
          push254Phase = Push254Phase.PUSHING;
          push254JamDebouncer.calculate(false);
          if (!Constants.MINIMAL_LOGGING) Logger.recordOutput("Superstructure/NoShotTrigger", true);
          intake.io.setIntakeArmVoltage(Volts.of(0));
          intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        }
        break;
      default:
        break;
    }
  }

  private boolean applyFeedingLogic() {
    // Jam detection: feeder velocity near zero while commanded to shoot
    double feederAbsVelocityRPS = Math.abs(shooter.inputs.feederVelocity.in(RotationsPerSecond));
    boolean potentiallyJammed =
        feederAbsVelocityRPS < ShooterConstants.JAM_FEEDER_MIN_VELOCITY_RPS.get()
            && jamCooldownTimer.hasElapsed(ShooterConstants.JAM_COOLDOWN_SECONDS);
    boolean jamConfirmed = jamDetectDebouncer.calculate(potentiallyJammed);

    if (jamConfirmed && !jamReverseActive) {
      jamReverseActive = true;
      jamReverseTimer.restart();
      if (!Constants.MINIMAL_LOGGING) Logger.recordOutput("Superstructure/JamDetected", true);
    } else {
      if (!Constants.MINIMAL_LOGGING) Logger.recordOutput("Superstructure/JamDetected", false);
    }

    if (jamReverseActive) {
      if (jamReverseTimer.hasElapsed(ShooterConstants.JAM_REVERSE_DURATION_SECONDS.get())) {
        jamReverseActive = false;
        jamCooldownTimer.restart();
      } else {
        shooter.setFeederVoltage(Volts.of(-12.0));
        hopper.setIndexerSpeed(RotationsPerSecond.of(-HopperConstants.INDEXER_SPEED.get()));
        hopper.setTopIndexerSpeed(RotationsPerSecond.of(-HopperConstants.TOP_INDEXER_SPEED.get()));
        return false;
      }
    }

    if (shooter.flywheelUpToSpeed() && !crawlUpScheduled) {
      crawlUpScheduled = true;
      oscTimer = 0.0;
      push254Phase = Push254Phase.PUSHING;
      push254JamDebouncer.calculate(false);
    }

    boolean beamBreak = shooter.isShooting();
    boolean beamBreakRisingEdge = beamBreak && !prevBeamBreak;

    boolean isFlowMode =
        ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.BEAM_HOPPER
            || ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.FUSED_HOPPER;

    if (!isFlowMode) {
      if (beamBreakRisingEdge) {
        intakeWaitingForNextBall = !intakeWaitingForNextBall;
        if (intakeWaitingForNextBall) {
          intakeDownTimer.restart();
          push254Phase = Push254Phase.PUSHING;
          push254JamDebouncer.calculate(false);
        }
      }
      if (intakeWaitingForNextBall && intakeDownTimer.hasElapsed(0.25)) {
        intakeWaitingForNextBall = false;
        push254Phase = Push254Phase.PUSHING;
        push254JamDebouncer.calculate(false);
      }
    }
    prevBeamBreak = beamBreak;

    if (crawlUpScheduled
        && wantedSuperState != WantedState.SHOOT_INTAKE
        && wantedSuperState != WantedState.PASS_INTAKE) {
      if (!isFlowMode && intakeWaitingForNextBall) {
        intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        if (!Constants.MINIMAL_LOGGING)
          Logger.recordOutput("Superstructure/IntakeLoweredForBall", true);
      } else {
        if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.PUSH_254) {
          apply254Push();
        } else if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.BEAM_HOPPER) {
          applyBeamHopperFeeding();
        } else if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.FUSED_HOPPER) {
          applyFusedHopperFeeding();
        } else {
          applyPivotOscillate();
        }
        if (!Constants.MINIMAL_LOGGING)
          Logger.recordOutput("Superstructure/IntakeLoweredForBall", false);
      }
    }

    simulateShot();
    return beamBreakRisingEdge;
  }

  private void applyPivotOscillate() {
    oscTimer += 0.02;

    double downAngle = IntakeConstants.INTAKE_DOWN_VALUE.get() + 30;
    double upAngle = IntakeConstants.INTAKE_UP_VALUE.get();
    double centerMin = downAngle + IntakeConstants.OSCILLATE_CENTER_OFFSET_DEGREES.get();
    double centerMax = upAngle - IntakeConstants.OSCILLATE_CENTER_OFFSET_DEGREES.get();

    // Center sweeps from centerMin up to centerMax at the configured rate, then holds
    double center =
        Math.min(centerMin + IntakeConstants.OSCILLATE_SWEEP_RATE_DPS.get() * oscTimer, centerMax);

    // Asymmetric oscillation around the moving center: dutyCycle fraction of each period is spent
    // above center (positive), (1-dutyCycle) fraction below. dutyCycle=0.5 → symmetric sine.
    double amplitude = IntakeConstants.OSCILLATE_AMPLITUDE_DEGREES.get();
    double frequency = IntakeConstants.OSCILLATE_FREQUENCY_HZ.get();
    double dutyCycle = Math.max(0.01, Math.min(0.99, IntakeConstants.OSCILLATE_DUTY_CYCLE.get()));
    double period = 1.0 / frequency;
    double phaseInPeriod = (oscTimer % period) / period; // 0..1 within current period
    double sinArg;
    if (phaseInPeriod < dutyCycle) {
      sinArg = Math.PI * phaseInPeriod / dutyCycle; // 0→π over the up portion
    } else {
      sinArg = Math.PI + Math.PI * (phaseInPeriod - dutyCycle) / (1.0 - dutyCycle); // π→2π
    }
    double setpoint = center + amplitude * Math.sin(sinArg);

    // Clamp to valid pivot range
    setpoint = Math.max(downAngle, Math.min(upAngle, setpoint));

    intake.setAngle(Degrees.of(setpoint));

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("Superstructure/OscillationCenter", center);
      Logger.recordOutput("Superstructure/OscillationSetpoint", setpoint);
    }
  }

  private void applyBeamHopperFeeding() {
    double timeSinceShot = shooter.getTimeSinceLastBeamBreakSec();
    boolean flowActive = timeSinceShot < ShooterConstants.FLOW_WINDOW_SECONDS.get();

    if (flowActive) {
      applyPivotOscillate();
    } else {
      intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
    }

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("Superstructure/FlowActive", flowActive);
      Logger.recordOutput("Superstructure/TimeSinceLastShot", timeSinceShot);
    }
  }

  private void applyFusedHopperFeeding() {
    double timeSinceShot = shooter.getTimeSinceLastBeamBreakSec();
    boolean flowActive = timeSinceShot < ShooterConstants.FLOW_WINDOW_SECONDS.get();
    boolean hopperJammed =
        hopper.inputs.indexerSupplyCurrent.in(Amps) > ShooterConstants.FUSED_JAM_CURRENT_AMPS.get();

    if (flowActive && !hopperJammed) {
      applyPivotOscillate();
    } else {
      intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
    }

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("Superstructure/FlowActive", flowActive);
      Logger.recordOutput("Superstructure/TimeSinceLastShot", timeSinceShot);
      Logger.recordOutput("Superstructure/HopperJammed", hopperJammed);
    }
  }

  private void apply254Push() {
    double ceilingDeg =
        IntakeConstants.INTAKE_UP_VALUE.get() - IntakeConstants.PUSH_254_TARGET_OFFSET_DEGREES;
    double positionDeg = intake.inputs.intakeArmPosition.in(Degrees);

    switch (push254Phase) {
      case PUSHING:
        intake.setAngle(Degrees.of(ceilingDeg));
        boolean nearTarget = positionDeg >= ceilingDeg - 2.0;
        if (nearTarget) {
          push254JamDebouncer.calculate(false);
        } else {
          boolean jammed =
              push254JamDebouncer.calculate(
                  Math.abs(intake.inputs.intakeArmVelocity.in(RotationsPerSecond))
                      < IntakeConstants.PUSH_254_JAM_VELOCITY_THRESHOLD_RPS);
          if (jammed) {
            push254Phase = Push254Phase.OSCILLATING;
            push254Timer.restart();
            oscTimer = 0.0;
          }
        }
        break;
      case OSCILLATING:
        applyPivotOscillate();
        if (push254Timer.hasElapsed(IntakeConstants.PUSH_254_OSCILLATE_DURATION_SECONDS)) {
          push254Phase = Push254Phase.RETRACTING;
          push254Timer.restart();
        }
        break;
      case RETRACTING:
        intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        if (push254Timer.hasElapsed(IntakeConstants.PUSH_254_RETRACT_DURATION_SECONDS)) {
          push254Phase = Push254Phase.PUSHING;
          push254JamDebouncer.calculate(false);
        }
        break;
    }

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("Superstructure/Push254Phase", push254Phase.toString());
      Logger.recordOutput("Superstructure/Push254CeilingDeg", ceilingDeg);
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

    if (!consumeFuelForShot.getAsBoolean()) return;

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
                            / 2.25),
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
    return shooter.flywheelUpToSpeed(ShooterConstants.BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT)
        && checkHubTolerance();
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

  private boolean isTooCloseToHub() {
    return drive
            .getPose()
            .getTranslation()
            .getDistance(Constants.HUB_POSE_3D.get().getTranslation().toTranslation2d())
        < ShooterConstants.MIN_SHOT_DISTANCE_METERS;
  }

  private double getHubRotationError() {
    return Math.abs(drive.getRotation().minus(shotSetpoint.robotPose.getRotation()).getDegrees());
  }

  public boolean checkHubTolerance() {
    return getHubRotationError() < getDynamicHubToleranceDegrees();
  }

  public boolean readyToShoot() {
    return shooter.flywheelUpToSpeed() && checkHubTolerance();
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
    if (wantedSuperState == WantedState.SHOOTING || wantedSuperState == WantedState.SHOOT_INTAKE)
      shotSetpoint = ShotPlanner.createShotSetpoint(drive.getPose(), drive.getChassisSpeeds());
    else if (wantedSuperState == WantedState.PASSING
        || wantedSuperState == WantedState.PASS_INTAKE) {
      shotSetpoint =
          ShotPlanner.createPassSetpoint(
              drive.getPose().getY() > Constants.LEFT_RIGHT_SPLIT
                  ? Constants.LEFT_TARGET_PASS_POSE2D.get()
                  : Constants.RIGHT_TARGET_PASS_POSE2D.get(),
              drive.getPose(),
              drive.getChassisSpeeds());
    } else if (wantedSuperState == WantedState.PIT_SHOOT) {
      shotSetpoint = new ShotSetpoint();
      shotSetpoint.flywheelSpeed =
          RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get());
      shotSetpoint.feederSpeed = RotationsPerSecond.of(ShooterConstants.FEEDER_SPEED.get());
    }

    handleWantedState();
    applyStates();

    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);

    if (!Constants.MINIMAL_LOGGING) {
      boolean shooterAtSpeed = shooter.flywheelUpToSpeed();
      boolean hubInTolerance = checkHubTolerance();
      Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterAtSpeed);
      Logger.recordOutput("Superstructure/HubRotationSetpoint", hubInTolerance);
      Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose);
      Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose());
      Logger.recordOutput(
          "Superstructure/HubRotationToleranceDeg", getDynamicHubToleranceDegrees());
      Logger.recordOutput(
          "Superstructure/HubRotationError",
          Math.abs(
              drive
                  .getPose()
                  .getRotation()
                  .minus(shotSetpoint.robotPose.getRotation())
                  .getDegrees()));
    }
  }

  @Override
  public void simulationPeriodic() {
    shotSimulationTime += 0.02;
  }
}
