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

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.AutoRoutines;
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
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.testmode.TestModeRunner;
import frc.robot.util.AlertUtils;
import frc.robot.util.AutomaticCommands;
import frc.robot.util.BatteryLogger;
import frc.robot.util.RobotBumpSim;
import frc.robot.util.RobotIdentity;
import java.util.function.BooleanSupplier;
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
  private final IntakeRoller intakeRoller;
  private final IntakePivot intakePivot;
  private final Superstructure superstructure;

  private final BatteryLogger batteryLogger = new BatteryLogger();

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation swerveDriveSimulation = null;
  private RobotBumpSim robotBumpSim = null;

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
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        shooter = new Shooter(new ShooterIOTalonFX());
        hopper = new Hopper(new HopperIOTalonFX());
        intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX(), () -> {});
        intakePivot = new IntakePivot(new IntakePivotIOTalonFX());
        break;

      case SIM:
        swerveDriveSimulation =
            MapleSimSwerve.createSimulationDrive(RobotIdentity.getTunerConstants());
        robotBumpSim = new RobotBumpSim(Drive.getModuleTranslations());
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
                    camera0Name, robotToCamera0, swerveDriveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    robotToCamera1,
                    swerveDriveSimulation::getSimulatedDriveTrainPose));
        shooter = new Shooter(new ShooterIOSim() {});
        hopper = new Hopper(new HopperIO() {});
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
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
        intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        break;
    }

    superstructure =
        new Superstructure(
            drive, shooter, hopper, intakePivot, intakeRoller, swerveDriveSimulation);

    drive.setBatteryLogger(batteryLogger);
    shooter.setBatteryLogger(batteryLogger);
    hopper.setBatteryLogger(batteryLogger);
    intakeRoller.setBatteryLogger(batteryLogger);
    intakePivot.setBatteryLogger(batteryLogger);

    // Set up auto routines
    LoggedDashboardChooser<Command> tempChooser;
    try {
      AutoRoutines autoRoutines = new AutoRoutines(drive, superstructure, intakePivot);
      tempChooser = new LoggedDashboardChooser<>("Auto Choices", autoRoutines.buildAutoChooser());
      tempChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      tempChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      tempChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      tempChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      tempChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      tempChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      tempChooser.addOption("Flywheel SysId", shooter.flywheelSysId());
    } catch (Exception e) {
      e.printStackTrace();
      Alert alert = new Alert("auto failed to load", AlertType.kError);
      alert.set(true);
      tempChooser = new LoggedDashboardChooser<>("Auto Choices");
    }
    autoChooser = tempChooser;

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive Command
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Reset Drive Rotation
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Point-opposite drive mode (hold right trigger)
    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDrivePointingOpposite(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> superstructure.getShotSetpoint().robotPose.getRotation()),
                    superstructure.setWantedSuperStateCommand(
                        () ->
                            Constants.SHOULD_PASS.apply(drive.getPose().getX())
                                ? Superstructure.WantedState.PASSING
                                : Superstructure.WantedState.SHOOTING))
                .until(() -> controller.leftBumper().getAsBoolean()))
        .onFalse(superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.IDLE));

    // Intake and Reverse Intake
    controller
        .leftTrigger()
        .onTrue(intakeRoller.reverseIntake())
        .onFalse(intakeRoller.stopIntake());
    controller.leftBumper().toggleOnTrue(superstructure.wantIntaking());

    // Intake Flip Position
    controller
        .a()
        .onTrue(
            Commands.either(
                intakePivot.setIntakeAngle(
                    () -> Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get())),
                intakePivot.setIntakeAngle(() -> Degrees.of(IntakeConstants.INTAKE_UP_VALUE.get())),
                () ->
                    intakePivot.targetAngle.equals(
                        Degrees.of(IntakeConstants.INTAKE_UP_VALUE.get()))));

    controller
        .x()
        .whileTrue(intakePivot.setIntakeArmVoltage(() -> Volts.of(-3)))
        .onFalse(intakePivot.setIntakeArmVoltage(() -> Volts.of(0)));

    // Intake Reset
    controller.back().onTrue(intakePivot.reset().ignoringDisable(true));
    new Trigger(() -> HALUtil.getFPGAButton()).onTrue(intakePivot.toggleBrakeMode());

    // Driver override: intentional stick movement cancels automation.
    // Uses a larger threshold than the drive deadband to avoid false triggers from
    // controller
    // drift.
    final double OVERRIDE_DEADBAND = 0.7;
    BooleanSupplier driverOverride =
        () ->
            Math.hypot(controller.getLeftX(), controller.getLeftY()) > OVERRIDE_DEADBAND
                || Math.abs(controller.getRightX()) > OVERRIDE_DEADBAND;

    // B button: context-aware trench / bump navigation
    controller.b().whileTrue(AutomaticCommands.automaticCommand(drive, driverOverride));

    // D-Pad: targeted automations
    controller.povUp().whileTrue(AutomaticCommands.hubBackWallCommand(drive, driverOverride));
    controller.povDown().whileTrue(AutomaticCommands.wallShootSetupCommand(drive, driverOverride));
    controller.povLeft().whileTrue(AutomaticCommands.underTowerCommand(drive, driverOverride));

    // D-Pad Right: shoot while drifting in -x direction
    controller
        .povRight()
        .whileTrue(
            Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -0.4,
                    () -> 0.0,
                    () -> superstructure.getShotSetpoint().robotPose.getRotation()),
                superstructure.setWantedSuperStateCommand(
                    () -> Superstructure.WantedState.SHOOTING)))
        .onFalse(superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.IDLE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class. f
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Returns the test-mode command. Called once per {@code testInit()}. */
  public Command getTestCommand() {
    return new TestModeRunner()
        .buildTestSequence(drive, shooter, hopper, intakeRoller, intakePivot);
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

    Pose2d simPose = swerveDriveSimulation.getSimulatedDriveTrainPose();
    var fieldSpeeds = swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    var simPose3d = robotBumpSim.update(simPose, fieldSpeeds, 5);

    if (robotBumpSim.isOnRamp()) {
      swerveDriveSimulation.setSimulationWorldPose(robotBumpSim.getSimWorldPose(simPose));
    }

    Logger.recordOutput("FieldSimulation/RobotPosition", simPose);
    Logger.recordOutput("Drive/Pose3d", simPose3d);
    Logger.recordOutput("Drive/isOnRamp", robotBumpSim.isOnRamp());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public BatteryLogger getBatteryLogger() {
    return batteryLogger;
  }

  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
