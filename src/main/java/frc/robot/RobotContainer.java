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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
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
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSetpoint;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AutoPilotUtils;
import frc.robot.util.RobotIdentity;
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
      new ShooterSetpoint(RPM.of(0), Degrees.of(0), new Pose2d(), new ChassisSpeeds());

  private LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("shooterspeed", 90);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontRight),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIOPhotonVision(camera0Name, robotToCamera0));
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
                new ModuleIOSim(RobotIdentity.getTunerConstants().FrontLeft),
                new ModuleIOSim(RobotIdentity.getTunerConstants().FrontRight),
                new ModuleIOSim(RobotIdentity.getTunerConstants().BackLeft),
                new ModuleIOSim(RobotIdentity.getTunerConstants().BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        objectDetection =
            new ObjectDetection(
                new ObjectDetectionIO() {}, (timestamp) -> drive.getTimestampPose(timestamp));
        shooter = new Shooter(new ShooterIOTalonFX() {});
        hopper = new Hopper(new HopperIO() {});
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
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

                              //   Check if within hub position and rotation tolerance
                              //   Pose2d currentPose = drive.getPose();
                              //   double positionError =
                              //       currentPose
                              //           .getTranslation()
                              //           .getDistance(shooterSetpoint.robotPose.getTranslation());
                              double rotationError =
                                  Math.abs(
                                      drive
                                          .getPose()
                                          .getRotation()
                                          .minus(shooterSetpoint.robotPose.getRotation())
                                          .getDegrees());
                              //   boolean hubPositionWithinTolerance =
                              //       positionError
                              //           < ShooterConstants.HUB_POSITION_TOLERANCE.in(Meters);
                              boolean hubRotationWithinTolerance =
                                  rotationError
                                      < ShooterConstants.HUB_ROTATION_TOLERANCE.in(Degrees);
                              Logger.recordOutput("hubtoleranceok", hubRotationWithinTolerance);

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
    return autoChooser.get();
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  //   public Vision getVisionSubsystem() {
  //     return vision;
  //   }

  public ObjectDetection getObjectDetectionSubsystem() {
    return objectDetection;
  }
}
