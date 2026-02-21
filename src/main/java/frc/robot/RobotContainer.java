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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AlertUtils;
import frc.robot.util.RobotIdentity;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation swerveDriveSimulation = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AlertUtils.criticalErrorRumbleFunction =
        () -> controller.setRumble(RumbleType.kBothRumble, 1.0);
    AlertUtils.stopRumbleFunction = () -> controller.setRumble(RumbleType.kBothRumble, 0.0);

    AlertUtils.clearCriticalAlerts();

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
        shooter = new Shooter(new ShooterIOTalonFX());
        hopper = new Hopper(new HopperIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        break;

      case SIM:
        swerveDriveSimulation =
            MapleSimSwerve.createSimulationDrive(RobotIdentity.getTunerConstants());
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
        shooter = new Shooter(new ShooterIOSim() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
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
        shooter = new Shooter(new ShooterIOTalonFX() {});
        hopper = new Hopper(new HopperIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }

    superstructure = new Superstructure(drive, shooter, hopper, swerveDriveSimulation);

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
    NamedCommands.registerCommand("Lower", intake.setIntakeAngle(() -> Degrees.of(0)));
    NamedCommands.registerCommand("StartIntake", intake.spinIntake());
    NamedCommands.registerCommand("StopIntake", intake.stopIntake());
    NamedCommands.registerCommand(
        "Shoot",
        Commands.sequence(
            intake.setIntakeAngle(() -> Degrees.of(45)),
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> 0,
                    () -> 0,
                    () -> superstructure.getShotSetpoint().robotPose.getRotation()),
                superstructure.setWantedSuperStateCommand(
                    () -> Superstructure.WantedState.SHOOTING))));
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
            () -> controller.getLeftX(),
            () -> -controller.getLeftY(),
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
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> controller.getLeftX(),
                        () -> -controller.getLeftY(),
                        () -> superstructure.getShotSetpoint().robotPose.getRotation()),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.PASSING))
                .until(() -> controller.leftBumper().getAsBoolean()))
        .onFalse(superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.IDLE));

    controller.leftTrigger().onTrue(intake.spinIntake()).onFalse(intake.stopIntake());
    controller.leftBumper().onTrue(intake.reverseIntake()).onFalse(intake.stopIntake());
    controller
        .a()
        .onTrue(intake.setIntakeAngle(() -> Degrees.of(IntakeConstants.INTAKE_UP_VALUE.get())))
        .onFalse(intake.setIntakeAngle(() -> Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get())));
    controller
        .b()
        .onTrue(intake.reset());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. f
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  public Vision getVisionSubsystem() {
    return vision;
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
