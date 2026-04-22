// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RobotIdentity;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(RobotIdentity.getTunerConstants().DrivetrainConstants.CANBusName).isNetworkFD()
          ? 250.0
          : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  RobotIdentity.getTunerConstants().FrontLeft.LocationX,
                  RobotIdentity.getTunerConstants().FrontLeft.LocationY),
              Math.hypot(
                  RobotIdentity.getTunerConstants().FrontRight.LocationX,
                  RobotIdentity.getTunerConstants().FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  RobotIdentity.getTunerConstants().BackLeft.LocationX,
                  RobotIdentity.getTunerConstants().BackLeft.LocationY),
              Math.hypot(
                  RobotIdentity.getTunerConstants().BackRight.LocationX,
                  RobotIdentity.getTunerConstants().BackRight.LocationY)));

  private static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(
            RobotIdentity.getTunerConstants().FrontLeft.LocationX,
            RobotIdentity.getTunerConstants().FrontLeft.LocationY),
        new Translation2d(
            RobotIdentity.getTunerConstants().FrontRight.LocationX,
            RobotIdentity.getTunerConstants().FrontRight.LocationY),
        new Translation2d(
            RobotIdentity.getTunerConstants().BackLeft.LocationX,
            RobotIdentity.getTunerConstants().BackLeft.LocationY),
        new Translation2d(
            RobotIdentity.getTunerConstants().BackRight.LocationX,
            RobotIdentity.getTunerConstants().BackRight.LocationY)
      };

  private static final SwerveModuleState[] EMPTY_MODULE_STATES = new SwerveModuleState[] {};

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = Pounds.of(135).in(Kilograms);
  private static final double ROBOT_MOI = 4.35;
  private static final double WHEEL_COF = 1.45;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              RobotIdentity.getTunerConstants().FrontLeft.WheelRadius,
              RobotIdentity.getTunerConstants().kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60(1)
                  .withReduction(RobotIdentity.getTunerConstants().FrontLeft.DriveMotorGearRatio),
              RobotIdentity.getTunerConstants().FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final SysIdRoutine sysIdSteer;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Pre-allocated return arrays for getModuleStates() / getModulePositions() — avoids per-call heap
  // allocation on every 20ms loop tick.
  private final SwerveModuleState[] moduleStateCache = new SwerveModuleState[4];
  private final SwerveModulePosition[] modulePositionCache = new SwerveModulePosition[4];

  // Pre-allocated odometry arrays — reused every cycle to avoid per-sample allocation.
  // Elements are initialized in the constructor and mutated in place each cycle.
  private final SwerveModulePosition[] odometryModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveModulePosition[] odometryModuleDeltas =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private boolean defenseModeActive = false;
  private boolean stopWithXRequested = false;
  private SwerveDriveSimulation sim;
  private final Field2d m_field = new Field2d();

  private BaseStatusSignal[] allSignals;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      SwerveDriveSimulation sim) {
    this.sim = sim;
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, RobotIdentity.getTunerConstants().FrontLeft);
    modules[1] = new Module(frModuleIO, 1, RobotIdentity.getTunerConstants().FrontRight);
    modules[2] = new Module(blModuleIO, 2, RobotIdentity.getTunerConstants().BackLeft);
    modules[3] = new Module(brModuleIO, 3, RobotIdentity.getTunerConstants().BackRight);

    // Build combined signal array for single-batch CAN refresh
    var gyroSignals = gyroIO.getSignals();
    var moduleSignals =
        java.util.Arrays.stream(new ModuleIO[] {flModuleIO, frModuleIO, blModuleIO, brModuleIO})
            .flatMap(io -> java.util.Arrays.stream(io.getSignals()))
            .toArray(BaseStatusSignal[]::new);
    allSignals =
        java.util.stream.Stream.concat(
                java.util.Arrays.stream(gyroSignals), java.util.Arrays.stream(moduleSignals))
            .toArray(BaseStatusSignal[]::new);

    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds, feedforwards),
        new PPHolonomicDriveController(
            new PIDConstants(5.6, 0.02, 0.35), new PIDConstants(5.2, 0.02, 0.35)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          if (!Constants.MINIMAL_LOGGING)
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          if (!Constants.MINIMAL_LOGGING)
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                (log) -> {
                  for (int i = 0; i < 4; i++) {
                    log.motor("drive-" + i)
                        .voltage(Volts.of(modules[i].getDriveAppliedVolts()))
                        .angularPosition(Radians.of(modules[i].getDrivePositionRad()))
                        .angularVelocity(
                            RadiansPerSecond.of(modules[i].getDriveVelocityRadPerSec()));
                  }
                },
                this));
    sysIdSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdSteerState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runSteerCharacterization(voltage.in(Volts)),
                (log) -> {
                  for (int i = 0; i < 4; i++) {
                    log.motor("steer-" + i)
                        .voltage(Volts.of(modules[i].getTurnAppliedVolts()))
                        .angularPosition(Radians.of(modules[i].getAngle().getRadians()))
                        .angularVelocity(
                            RadiansPerSecond.of(modules[i].getSteerVelocityRadPerSec()));
                  }
                },
                this));

    setpointGenerator =
        new SwerveSetpointGenerator(
            PP_CONFIG,
            DriveConstants.MAX_MODULE_ANGULAR_VELOCITY); // Adjust maxSteerVelocity if needed
    previousSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
  }

  @Override
  public void periodic() {
    double periodicStartSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Refresh all CAN signals in one batch before acquiring the lock.
    // This replaces N sequential blocking waitForAll calls with a single one.
    if (allSignals.length > 0) {
      BaseStatusSignal.refreshAll(allSignals);
    }

    // Under lock: hardware reads only. Logger.processInputs is intentionally outside
    // the lock — it reads from already-captured structs and doesn't need protection.
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateHardwareInputs();
    }
    odometryLock.unlock();

    // Outside lock: AK serialization + alert/odometry updates
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.logAndProcessInputs();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled() && !Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("SwerveStates/Setpoints", EMPTY_MODULE_STATES);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", EMPTY_MODULE_STATES);
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module, mutating pre-allocated objects
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        SwerveModulePosition odometryPos = modules[moduleIndex].getOdometryPositions()[i];
        odometryModuleDeltas[moduleIndex].distanceMeters =
            odometryPos.distanceMeters - lastModulePositions[moduleIndex].distanceMeters;
        odometryModuleDeltas[moduleIndex].angle = odometryPos.angle;
        odometryModulePositions[moduleIndex].distanceMeters = odometryPos.distanceMeters;
        odometryModulePositions[moduleIndex].angle = odometryPos.angle;
        lastModulePositions[moduleIndex].distanceMeters = odometryPos.distanceMeters;
        lastModulePositions[moduleIndex].angle = odometryPos.angle;
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(odometryModuleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, odometryModulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(getPose());

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput(
          "Drive/PeriodicTimeMs",
          (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - periodicStartSec) * 1000.0);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, DriveFeedforwards.zeros(4));
  }

  public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    SwerveModuleState[] setpointStates;

    boolean isZeroSpeeds =
        speeds.vxMetersPerSecond == 0.0
            && speeds.vyMetersPerSecond == 0.0
            && speeds.omegaRadiansPerSecond == 0.0;

    if (stopWithXRequested && isZeroSpeeds) {
      // Apply X formation directly, bypassing setpoint generator
      Rotation2d[] headings = new Rotation2d[4];
      setpointStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        headings[i] = getModuleTranslations()[i].getAngle();
        setpointStates[i] = new SwerveModuleState(0.0, headings[i]);
      }
      kinematics.resetHeadings(headings);
      previousSetpoint =
          new SwerveSetpoint(new ChassisSpeeds(), setpointStates, DriveFeedforwards.zeros(4));
    } else {
      if (!isZeroSpeeds) stopWithXRequested = false;
      if (DriveConstants.BYPASS_SETPOINT_GENERATOR) {
        setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            setpointStates, getMaxLinearSpeedMetersPerSec());
      } else {
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
        setpointStates = previousSetpoint.moduleStates();
      }
    }

    if (!Constants.MINIMAL_LOGGING) {
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput(
          "SwerveChassisSpeeds/Setpoints",
          DriveConstants.BYPASS_SETPOINT_GENERATOR
              ? speeds
              : previousSetpoint.robotRelativeSpeeds());
      Logger.recordOutput(
          "Drive/BypassSetpointGenerator", DriveConstants.BYPASS_SETPOINT_GENERATOR);
    }

    // Copy each state so that Module.runSetpoint's optimize() mutation doesn't corrupt
    // setpointStates, which would cause 180° flip oscillation on the next cycle.
    double[] torqueCurrents = feedforwards.torqueCurrentsAmps();
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(
          new SwerveModuleState(setpointStates[i].speedMetersPerSecond, setpointStates[i].angle),
          torqueCurrents[i]);
    }

    if (!Constants.MINIMAL_LOGGING)
      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    stopWithXRequested = true;
    stop();
    Logger.recordOutput("Drive/StopWithX", true);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Runs all steer motors open-loop for SysId characterization. */
  private void runSteerCharacterization(double output) {
    for (var module : modules) {
      module.runSteerCharacterization(output);
    }
  }

  /** Returns a command to run a steer quasistatic SysId test in the specified direction. */
  public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runSteerCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdSteer.quasistatic(direction));
  }

  /** Returns a command to run a steer dynamic SysId test in the specified direction. */
  public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runSteerCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysIdSteer.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < 4; i++) {
      moduleStateCache[i] = modules[i].getState();
    }
    return moduleStateCache;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      modulePositionCache[i] = modules[i].getPosition();
    }
    return modulePositionCache;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the drive motor velocity in rad/s for the given module index. Used by test mode. */
  public double getModuleDriveVelocityRadPerSec(int index) {
    return modules[index].getDriveVelocityRadPerSec();
  }

  /** Returns the steer motor velocity in rad/s for the given module index. Used by test mode. */
  public double getModuleSteerVelocityRadPerSec(int index) {
    return modules[index].getSteerVelocityRadPerSec();
  }

  /** Runs all steer motors open-loop. Used by test mode. */
  public void runSteerOpenLoop(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runSteerOpenLoop(output);
    }
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the pose at a given timestamp. */
  public Optional<Pose2d> getTimestampPose(double timestamp) {
    return poseEstimator.sampleAt(timestamp);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);

    if (Constants.currentMode == Mode.SIM) {
      sim.setSimulationWorldPose(pose);
    }
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return RobotIdentity.getTunerConstants().kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns the cached array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return MODULE_TRANSLATIONS;
  }

  public Command pathfindToPoseFaced(
      Pose2d targetPose, PathConstraints constraints, Supplier<Translation2d> aimTargetSupplier) {

    if (!Constants.MINIMAL_LOGGING) Logger.recordOutput("Pathfinding/targetPose", targetPose);

    // Initialize with null to indicate we haven't latched yet
    AtomicReference<Double> latchedYaw = new AtomicReference<>(null);

    return new PathfindingCommand(
        targetPose,
        constraints,
        0.0,
        this::getPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> {
          Pose2d currentPose = getPose();
          Translation2d aimTarget = aimTargetSupplier.get();
          double distance = currentPose.getTranslation().getDistance(aimTarget);

          double targetYaw;

          // Threshold: 0.25 meters (approx 10 inches)
          if (distance < 0.4) {
            // If we haven't latched yet, calculate and store the current yaw
            if (latchedYaw.get() == null) {
              latchedYaw.set(
                  Math.atan2(
                      aimTarget.getY() - currentPose.getY(),
                      aimTarget.getX() - currentPose.getX()));
            }
            targetYaw = latchedYaw.get();
          } else {
            // Normal dynamic tracking
            targetYaw =
                Math.atan2(
                    aimTarget.getY() - currentPose.getY(), aimTarget.getX() - currentPose.getX());
            // Clear latch if we move away (e.g., target moved or robot bumped)
            latchedYaw.set(null);
          }

          double currentYaw = currentPose.getRotation().getRadians();
          double angularError = MathUtil.angleModulus(targetYaw - currentYaw);

          // Rotation P-Loop
          double omega = angularError * 4.15;
          omega =
              MathUtil.clamp(omega, -getMaxAngularSpeedRadPerSec(), getMaxAngularSpeedRadPerSec());

          ChassisSpeeds overriddenSpeeds =
              new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);

          runVelocity(overriddenSpeeds);
        },
        new PPHolonomicDriveController(
            new PIDConstants(4.5, 0.02, 0.1), new PIDConstants(4.5, 0.02, 0.1)),
        PP_CONFIG,
        this);
  }

  public Command pathfindToPoseFaced(
      Pose2d targetPose,
      PathConstraints constraints,
      Supplier<Translation2d> aimTargetSupplier,
      double goalEndVelocity) {

    if (!Constants.MINIMAL_LOGGING) Logger.recordOutput("Pathfinding/targetPose", targetPose);

    return new PathfindingCommand(
        targetPose,
        constraints,
        goalEndVelocity, // Goal end velocity
        this::getPose,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> {
          if (!Constants.MINIMAL_LOGGING) Logger.recordOutput("Pathfinding/speeds", speeds);

          Pose2d currentPose = getPose();
          Translation2d aimTarget = aimTargetSupplier.get();
          double targetYaw =
              Math.atan2(
                  aimTarget.getY() - currentPose.getY(), aimTarget.getX() - currentPose.getX());

          double currentYaw = currentPose.getRotation().getRadians();
          double angularError = MathUtil.angleModulus(targetYaw - currentYaw);
          double omega = angularError * 4.15;

          omega =
              MathUtil.clamp(omega, -getMaxAngularSpeedRadPerSec(), getMaxAngularSpeedRadPerSec());

          ChassisSpeeds overriddenSpeeds =
              new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);

          runVelocity(overriddenSpeeds);
        },
        new PPHolonomicDriveController(
            new PIDConstants(4.5, 0.02, 0.1), // Translation PID (Matches AutoBuilder)
            new PIDConstants(
                4.0, 0.02, 0.1) // Internal Rotation PID (Not used for aiming, but required)
            ),
        PP_CONFIG,
        this);
  }

  /**
   * Drives at a constant field-relative velocity until the robot stalls (wall contact) or the
   * driver overrides. A debouncer prevents the stall check from firing before the robot has had a
   * chance to accelerate.
   */
  public Command driveIntoWall(double fieldVx, double fieldVy, BooleanSupplier driverOverride) {
    Debouncer stallDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kRising);
    return this.run(
            () -> {
              runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      fieldVx, fieldVy, 0, getPose().getRotation()));
              if (!Constants.MINIMAL_LOGGING) {
                ChassisSpeeds actual = getChassisSpeeds();
                Logger.recordOutput(
                    "DriveIntoWall/actualSpeed",
                    Math.hypot(actual.vxMetersPerSecond, actual.vyMetersPerSecond));
              }
            })
        .until(
            () -> {
              ChassisSpeeds actual = getChassisSpeeds();
              boolean stalled =
                  Math.hypot(actual.vxMetersPerSecond, actual.vyMetersPerSecond) < 0.05;
              return stallDebouncer.calculate(stalled) || driverOverride.getAsBoolean();
            })
        .finallyDo(() -> runVelocity(new ChassisSpeeds()));
  }

  /**
   * Drives at constant robot-relative speeds until the robot has traveled the given distance
   * (measured via odometry) or the driver overrides.
   */
  public Command driveForDistance(
      ChassisSpeeds robotRelative, double distanceMeters, BooleanSupplier driverOverride) {
    return Commands.defer(
        () -> {
          Translation2d start = getPose().getTranslation();
          return this.run(() -> runVelocity(robotRelative))
              .until(
                  () ->
                      getPose().getTranslation().getDistance(start) >= distanceMeters
                          || driverOverride.getAsBoolean())
              .finallyDo(() -> runVelocity(new ChassisSpeeds()));
        },
        Set.of(this));
  }

  private static final APConstraints kConstraints =
      new APConstraints().withVelocity(3.5).withAcceleration(8.0);

  private static final APProfile kProfile =
      new APProfile(kConstraints)
          .withErrorXY(Centimeters.of(6))
          .withErrorTheta(Degrees.of(4))
          .withBeelineRadius(Centimeters.of(10));

  public static final Autopilot kAutopilot = new Autopilot(kProfile);

  /**
   * Toggles defense mode. When active, drive motor current limits are raised for pushing power;
   * other subsystems lower their limits via their own setDefenseMode() calls.
   */
  public void setDefenseMode(boolean active) {
    if (active == defenseModeActive) return;
    defenseModeActive = active;
    for (var module : modules) {
      // Normal:  supply 55 A / stator 102 A (kSlipCurrent)
      // Defense: supply 80 A / stator 120 A
      module.setDriveCurrentLimits(active ? 80.0 : 55.0, active ? 120.0 : 102.0);
    }
    Logger.recordOutput("Drive/DefenseMode", defenseModeActive);
  }

  /** Returns whether defense mode is currently active. */
  public boolean isDefenseModeActive() {
    return defenseModeActive;
  }
}
