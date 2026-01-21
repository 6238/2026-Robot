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

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.MapleSimSwerve;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.objectdetection.ObjectDetection;
import frc.robot.subsystems.objectdetection.ObjectDetectionIO;
import frc.robot.subsystems.objectdetection.ObjectDetectionIOJetson;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSetpoint;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AutoPilotUtils;
import frc.robot.util.RobotIdentity;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.*;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final ObjectDetection objectDetection;
  private final Shooter shooter;
  private final Hopper hopper;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private ShooterSetpoint shooterSetpoint =
      new ShooterSetpoint(
          RPM.of(0), Degrees.of(0), new Pose2d(), new ChassisSpeeds(), new Translation2d());

  private LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("shooterspeed", 90);

  private SwerveDriveSimulation swerveDriveSimulation;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerveDriveSimulation = MapleSimSwerve.createSimulationDrive(RobotIdentity.getTunerConstants());
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontRight),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackRight),
                swerveDriveSimulation);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVision(camera0Name, robotToCamera0));
        objectDetection =
            new ObjectDetection(
                new ObjectDetectionIOJetson(), (timestamp) -> drive.getTimestampPose(timestamp));
        shooter = new Shooter(new ShooterIOTalonFX());
        hopper = new Hopper(new HopperIOTalonFX());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
                swerveDriveSimulation);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    camera0Name,
                    robotToCamera0,
                    swerveDriveSimulation::getSimulatedDriveTrainPose));
        objectDetection =
            new ObjectDetection(
                new ObjectDetectionIO() {}, (timestamp) -> drive.getTimestampPose(timestamp));
        shooter = new Shooter(new ShooterIOSim() {});
        hopper = new Hopper(new HopperIO() {});
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                swerveDriveSimulation);
        vision = new Vision(drive::addVisionMeasurement, drive::getPose, new VisionIO() {});
        objectDetection =
            new ObjectDetection(
                new ObjectDetectionIO() {}, (timestamp) -> drive.getTimestampPose(timestamp));
        shooter = new Shooter(new ShooterIOTalonFX() {});
        hopper = new Hopper(new HopperIO() {});
        break;
    }

    configureNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureNamedCommands() {
    AutoPilotUtils.initializeAutoPilot();

    NamedCommands.registerCommand(
        "DynamicPickup",
        Commands.either(
            AutoPilotUtils.generateIterativePickupCommand(drive, objectDetection, controller),
            Commands.sequence(
                Commands.waitSeconds(0.3),
                Commands.either(
                    AutoPilotUtils.generateIterativePickupCommand(
                        drive, objectDetection, controller),
                    Commands.none(),
                    () -> objectDetection.getTrackedObjects().length > 0)),
            () -> objectDetection.getTrackedObjects().length > 0));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () ->
                        shooterSetpoint =
                            ShooterSetpoint.generateNearestShooterSetpoint(
                                drive.getPose(), drive.getChassisSpeeds())),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> controller.getLeftY(),
                    () -> controller.getLeftX(),
                    () -> shooterSetpoint.robotPose.getRotation()),
                Commands.sequence(
                    shooter.setFeederVoltage(() -> Volts.of(ShooterConstants.FEEDER_VOLTAGE.get())),
                    Commands.repeatingSequence(
                        shooter.setFlywheelRPM(() -> shooterSetpoint.flywheelSpeed))),
                Commands.repeatingSequence(
                    hopper
                        .spinIndexer()
                        .onlyIf(
                            () -> {
                              Logger.recordOutput(
                                  "Shooter/Setpoint/FlywheelSpeed",
                                  shooterSetpoint.flywheelSpeed.in(RotationsPerSecond));
                              Logger.recordOutput(
                                  "Shooter/Setpoint/FlywheelCurrentSpeed",
                                  shooter.getCurrentFlywheelSpeed().in(RotationsPerSecond));

                              Logger.recordOutput(
                                  "Shooter/Setpoint/HoodAngle", shooterSetpoint.hoodAngle);
                              Logger.recordOutput(
                                  "Shooter/Setpoint/RobotPose", shooterSetpoint.robotPose);
                              Logger.recordOutput(
                                  "Shooter/Setpoint/RobotSpeeds", shooterSetpoint.robotSpeeds);

                              // Check if within shot tolerance
                              AngularVelocity currentRPM = shooter.getCurrentFlywheelSpeed();
                              AngularVelocity targetRPM = shooterSetpoint.flywheelSpeed;
                              double rpmError =
                                  Math.abs(
                                      currentRPM.in(RevolutionsPerSecond)
                                          - targetRPM.in(RevolutionsPerSecond));
                              boolean shooterRPMWithinTolerance =
                                  rpmError
                                      < ShooterConstants.FLYWHEEL_TOLERANCE_BEFORE_SHOT.in(
                                          RevolutionsPerSecond);
                              Logger.recordOutput("oktoshoot", shooterRPMWithinTolerance);

                              // Check if within hub position and rotation tolerance
                              // Pose2d currentPose = drive.getPose();
                              // double positionError =
                              // currentPose
                              // .getTranslation()
                              // .getDistance(shooterSetpoint.robotPose.getTranslation());
                              double rotationError =
                                  Math.abs(
                                      drive
                                          .getPose()
                                          .getRotation()
                                          .minus(shooterSetpoint.robotPose.getRotation())
                                          .getDegrees());
                              // boolean hubPositionWithinTolerance =
                              // positionError
                              // < ShooterConstants.HUB_POSITION_TOLERANCE.in(Meters);
                              boolean hubRotationWithinTolerance =
                                  Math.abs(rotationError)
                                      < ShooterConstants.HUB_ROTATION_TOLERANCE.in(Degrees);
                              Logger.recordOutput("HUB_OK", hubRotationWithinTolerance);

                              return shooterRPMWithinTolerance && hubRotationWithinTolerance;
                            }))))
        .onFalse(
            Commands.sequence(
                hopper.stopIndexer(),
                shooter.setFlywheelRPM(() -> RPM.of(0)),
                shooter.setFeederVoltage(() -> Volts.of(0))));

    controller
        .a()
        .onTrue(shooter.setFeederVoltage(() -> Volts.of(-5)))
        .onFalse(shooter.setFeederVoltage(() -> Volts.of(0)));
    controller.b().onTrue(hopper.reverseIndexer()).onFalse(hopper.stopIndexer());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile("test");
    } catch (Exception e) {
      DataLogManager.log(e.getMessage());
      path = new PathPlannerPath(null, null, null, null);
    }
    Pose2d startPose2d = path.getStartingHolonomicPose().orElse(new Pose2d(0, 0, Rotation2d.kZero));
    Logger.recordOutput("startpose", startPose2d);

    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(startPose2d), drive),
        Commands.parallel(
            Commands.run(
                () ->
                    shooterSetpoint =
                        ShooterSetpoint.generateNearestShooterSetpoint(
                            drive.getPose(), drive.getChassisSpeeds())),
            DriveCommands.followPathWhileAiming(
                drive,
                path,
                () -> {
                  return shooterSetpoint.target;
                }),
            Commands.sequence(
                shooter.setFeederVoltage(() -> Volts.of(ShooterConstants.FEEDER_VOLTAGE.get())),
                Commands.repeatingSequence(
                    shooter.setFlywheelRPM(() -> shooterSetpoint.flywheelSpeed))),
            Commands.repeatingSequence(
                Commands.parallel(
                        hopper.spinIndexer(),
                        Commands.runOnce(
                                () -> {
                                  SimulatedArena.getInstance()
                                      .addGamePieceProjectile(
                                          new RebuiltFuelOnFly(
                                                  swerveDriveSimulation
                                                      .getSimulatedDriveTrainPose()
                                                      .getTranslation(),
                                                  new Translation2d(Units.inchesToMeters(12), 0),
                                                  swerveDriveSimulation
                                                      .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                                  swerveDriveSimulation
                                                      .getSimulatedDriveTrainPose()
                                                      .getRotation(),
                                                  Inches.of(21),
                                                  MetersPerSecond.of(
                                                      shooterSetpoint.flywheelSpeed.in(
                                                              RotationsPerSecond)
                                                          * 2
                                                          * 0.0508
                                                          * Math.PI
                                                          / 1.9),
                                                  ShooterConstants.FIXED_HOOD_ANGLE_DEGREES)
                                              .withProjectileTrajectoryDisplayCallBack(
                                                  // Callback for when the note will eventually hit
                                                  // the
                                                  // target (if configured)
                                                  (pose3ds) ->
                                                      Logger.recordOutput(
                                                          "Flywheel/NoteProjectileSuccessfulShot",
                                                          pose3ds.toArray(Pose3d[]::new)),
                                                  // Callback for when the note will eventually miss
                                                  // the
                                                  // target, or if no target is configured
                                                  (pose3ds) ->
                                                      Logger.recordOutput(
                                                          "Flywheel/NoteProjectileUnsuccessfulShot",
                                                          pose3ds.toArray(Pose3d[]::new)))
                                              .withTargetPosition(
                                                  () ->
                                                      FieldMirroringUtils
                                                          .toCurrentAllianceTranslation(
                                                              new Translation3d(1.52, 4.11, 1.83)))
                                              // Set the tolerance: x: ±0.52m, y: ±0.52m, z: ±0.25m
                                              // (hexagonal funnel opening ~41 inches wide)
                                              .withTargetTolerance(
                                                  new Translation3d(0.52, 0.52, 0.25))
                                              // Set a callback to run when fuel hits the target
                                              .withHitTargetCallBack(
                                                  () ->
                                                      System.out.println(
                                                          "Scored fuel in Hub, +1 point!")));
                                })
                            .onlyIf(
                                () -> (Constants.currentMode == Mode.SIM) && Math.random() < 0.10))
                    .onlyIf(
                        () -> {
                          Logger.recordOutput(
                              "Shooter/Setpoint/FlywheelSpeed",
                              shooterSetpoint.flywheelSpeed.in(RotationsPerSecond));
                          Logger.recordOutput(
                              "Shooter/Setpoint/FlywheelCurrentSpeed",
                              shooter.getCurrentFlywheelSpeed().in(RotationsPerSecond));

                          Logger.recordOutput(
                              "Shooter/Setpoint/HoodAngle", shooterSetpoint.hoodAngle);
                          Logger.recordOutput(
                              "Shooter/Setpoint/RobotPose", shooterSetpoint.robotPose);
                          Logger.recordOutput(
                              "Shooter/Setpoint/RobotSpeeds", shooterSetpoint.robotSpeeds);

                          // Check if within shot tolerance
                          AngularVelocity currentRPM = shooter.getCurrentFlywheelSpeed();
                          AngularVelocity targetRPM = shooterSetpoint.flywheelSpeed;
                          double rpmError =
                              Math.abs(
                                  currentRPM.in(RevolutionsPerSecond)
                                      - targetRPM.in(RevolutionsPerSecond));
                          boolean shooterRPMWithinTolerance =
                              rpmError
                                  < ShooterConstants.FLYWHEEL_TOLERANCE_BEFORE_SHOT.in(
                                      RevolutionsPerSecond);
                          Logger.recordOutput("oktoshoot", shooterRPMWithinTolerance);

                          // Check if within hub position and rotation tolerance
                          // Pose2d currentPose = drive.getPose();
                          // double positionError =
                          // currentPose
                          // .getTranslation()
                          // .getDistance(shooterSetpoint.robotPose.getTranslation());
                          double rotationError =
                              Math.abs(
                                  drive
                                      .getPose()
                                      .getRotation()
                                      .minus(shooterSetpoint.robotPose.getRotation())
                                      .getDegrees());
                          // boolean hubPositionWithinTolerance =
                          // positionError
                          // < ShooterConstants.HUB_POSITION_TOLERANCE.in(Meters);
                          boolean hubRotationWithinTolerance =
                              rotationError < ShooterConstants.HUB_ROTATION_TOLERANCE.in(Degrees);
                          Logger.recordOutput("HUBERROR", rotationError);

                          return shooterRPMWithinTolerance; // && hubRotationWithinTolerance;
                        }))),
        Commands.sequence(
            hopper.stopIndexer(),
            shooter.setFlywheelRPM(() -> RPM.of(0)),
            shooter.setFeederVoltage(() -> Volts.of(0))));
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  // public Vision getVisionSubsystem() {
  // return vision;
  // }

  public ObjectDetection getObjectDetectionSubsystem() {
    return objectDetection;
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
