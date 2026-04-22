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
import frc.robot.subsystems.GratuitousLighting;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.MapleSimSwerve;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.WantedState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.testmode.TestModeRunner;
import frc.robot.util.AlertUtils;
import frc.robot.util.AutomaticCommands;
import frc.robot.util.RobotBumpSim;
import frc.robot.util.RobotIdentity;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.IntakeSimulation;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private boolean defenseModeActive = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<String> testModeChooser;

  private SwerveDriveSimulation swerveDriveSimulation = null;
  private RobotBumpSim robotBumpSim = null;
  private IntakeSimulation fuelIntake = null;
  public GratuitousLighting lighting = new GratuitousLighting();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    AlertUtils.criticalErrorRumbleFunction = () -> controller.setRumble(RumbleType.kBothRumble, 0);
    // () -> controller.setRumble(RumbleType.kBothRumble, 1.0);
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
        // objectDetection =
        //     new ObjectDetection(new ObjectDetectionIOJetson("objectdetection"), drive::getPose);
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
        hopper = new Hopper(new HopperIOSim());
        intakeRoller = new IntakeRoller(new IntakeRollerIOSim(), () -> {});
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        // objectDetection =
        //     new ObjectDetection(
        //         new ObjectDetectionIOSim(swerveDriveSimulation::getSimulatedDriveTrainPose),
        //         drive::getPose);
        fuelIntake =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                swerveDriveSimulation,
                Meters.of(0.7),
                Inches.of(12),
                IntakeSimulation.IntakeSide.BACK,
                40);
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
        // objectDetection = new ObjectDetection(new ObjectDetectionIO() {}, drive::getPose);
        break;
    }

    superstructure =
        new Superstructure(
            drive, shooter, hopper, intakePivot, intakeRoller, swerveDriveSimulation);

    if (fuelIntake != null) {
      superstructure.consumeFuelForShot = fuelIntake::obtainGamePieceFromIntake;
    }

    intakeRoller.setPivotAngleSupplier(() -> intakePivot.inputs.intakeArmPosition.in(Degrees));
    superstructure.hubSpinupActive = () -> shouldSpinupFlywheel(DriverStation.getMatchTime());
    lighting.superState = () -> superstructure.currentSuperState;

    // Set up auto routinesw
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
      tempChooser.addOption(
          "Steer SysId (Quasistatic Forward)",
          drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
      tempChooser.addOption(
          "Steer SysId (Quasistatic Reverse)",
          drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
      tempChooser.addOption(
          "Steer SysId (Dynamic Forward)",
          drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
      tempChooser.addOption(
          "Steer SysId (Dynamic Reverse)",
          drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));
    } catch (Exception e) {
      e.printStackTrace();
      Alert alert = new Alert("auto failed to load", AlertType.kError);
      alert.set(true);
      tempChooser = new LoggedDashboardChooser<>("Auto Choices");
    }
    autoChooser = tempChooser;

    testModeChooser = new LoggedDashboardChooser<>("Test Mode");
    testModeChooser.addDefaultOption("Normal", "Normal");
    testModeChooser.addOption("Baseline", "Baseline");
    testModeChooser.addOption("Ball Path", "Ball Path");

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive Command — trench Y-align always on; hold B to disable
    drive.setDefaultCommand(
        AutomaticCommands.trenchAwareJoystickDrive(
            drive,
            intakePivot,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .povRight()
        .onTrue(
            shooter.setFlywheelRPM(
                () -> RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get())));

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

    // Right trigger: shoot/pass with intake down and spinning
    controller
        .rightTrigger()
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
                                ? Superstructure.WantedState.PASS_INTAKE
                                : Superstructure.WantedState.SHOOT_INTAKE))
                .until(() -> controller.leftBumper().getAsBoolean()))
        .onFalse(superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.IDLE));
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
        .onTrue(
            Commands.parallel(
                intakeRoller.reverseIntake(),
                Commands.runOnce(
                    () ->
                        hopper.setTopIndexerSpeed(
                            RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get()))),
                Commands.runOnce(
                    () ->
                        hopper.setIndexerSpeed(
                            RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get())))))
        .onFalse(
            Commands.parallel(
                intakeRoller.stopIntake(),
                Commands.runOnce(() -> hopper.setTopIndexerSpeed(RotationsPerSecond.of(0))),
                Commands.runOnce(() -> hopper.setIndexerSpeed(RotationsPerSecond.of(0)))));
    controller.leftBumper().toggleOnTrue(superstructure.wantIntaking());

    // Intake Flip Position
    controller
        .a()
        .onTrue(
            Commands.either(
                intakePivot.setIntakeAngle(
                    () -> Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get())),
                intakePivot.setIntakeAngle(() -> IntakeConstants.INTAKE_START_VALUE),
                () -> intakePivot.targetAngle.equals(IntakeConstants.INTAKE_START_VALUE)));

    // X button binding moved below driverOverride definition

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

    // B button: continuous ball intake — follow the detected ball cluster path while intaking,
    // replanning automatically as new detections arrive. Driver stick movement cancels.
    // controller
    //     .b()
    //     .whileTrue(
    //         Commands.parallel(
    //                 objectDetection.continuousBallIntakeCommand(drive),
    //                 superstructure.wantIntaking())
    //             .until(driverOverride))
    //     .onFalse(superstructure.setWantedSuperStateCommand(() ->
    // Superstructure.WantedState.IDLE));

    // B button: pathfind to trench start, press intake into outer wall, then drive back at 1.0 m/s
    controller.b().whileTrue(AutomaticCommands.automaticCommand(drive, () -> false));
    controller.x().whileTrue(AutomaticCommands.neutralToAllianceCommand(drive, () -> false));

    // D-Pad: targeted automations
    // controller.povUp().whileTrue(AutomaticCommands.hubBackWallCommand(drive, driverOverride));
    // controller.povDown().whileTrue(AutomaticCommands.wallShootSetupCommand(drive,
    // driverOverride));
    // controller.povLeft().whileTrue(AutomaticCommands.underTowerCommand(drive, driverOverride));
    controller
        .povLeft()
        .whileTrue(
            Commands.startEnd(
                () -> superstructure.setWantedSuperState(Superstructure.WantedState.PIT_SHOOT),
                () -> superstructure.setWantedSuperState(Superstructure.WantedState.IDLE),
                superstructure));

    // Y button: toggle defense mode (raises drive current, lowers other subsystem current)
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  defenseModeActive = !defenseModeActive;
                  drive.setDefenseMode(defenseModeActive);
                  shooter.setDefenseMode(defenseModeActive);
                  hopper.setDefenseMode(defenseModeActive);
                  intakeRoller.setDefenseMode(defenseModeActive);
                  intakePivot.setDefenseMode(defenseModeActive);
                }));
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
    lighting.setTestResult(null);
    String mode = testModeChooser.get();
    TestModeRunner runner = new TestModeRunner();
    if ("Ball Path".equals(mode)) {
      return runner
          .buildBallPathSequence(shooter, hopper, intakeRoller, intakePivot)
          .andThen(Commands.runOnce(() -> lighting.setTestResult(true)));
    }
    return runner.buildTestSequence(
        drive,
        shooter,
        hopper,
        intakeRoller,
        intakePivot,
        "Baseline".equals(mode),
        lighting::setTestResult);
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

    if (fuelIntake != null) {
      if (superstructure.currentSuperState != Superstructure.CurrentState.IDLE) {
        fuelIntake.startIntake();
      } else {
        fuelIntake.stopIntake();
      }
      Logger.recordOutput("Simulation/HopperFuelCount", fuelIntake.getGamePiecesAmount());
    }

    Logger.recordOutput("FieldSimulation/RobotPosition", simPose);
    Logger.recordOutput("Drive/Pose3d", simPose3d);
    Logger.recordOutput("Drive/isOnRamp", robotBumpSim.isOnRamp());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  /**
   * Returns whether the hub is currently active for our alliance. Mirrors the WPILib game data
   * spec: game data char = alliance whose hub goes inactive first (active in Shifts 2 & 4).
   */
  public static boolean isHubActive(double matchTime) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return false;
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) return true; // data not yet received; assume active

    boolean redInactiveFirst =
        switch (gameData.charAt(0)) {
          case 'R' -> true;
          case 'B' -> false;
          default -> {
            yield true;
          } // invalid data; assume active
        };

    // Named alliance (goes inactive first) is active in Shifts 2 & 4.
    // shift1ActiveForUs = true  → our hub is on in Shifts 1 & 3
    // shift1ActiveForUs = false → our hub is on in Shifts 2 & 4
    boolean shift1ActiveForUs =
        alliance.get() == Alliance.Red ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > 130) return true;
    if (matchTime > 105) return shift1ActiveForUs;
    if (matchTime > 80) return !shift1ActiveForUs;
    if (matchTime > 55) return shift1ActiveForUs;
    if (matchTime > 30) return !shift1ActiveForUs;
    return true; // end game
  }

  /**
   * Returns true when the flywheel should pre-spin: hub is active OR within 2 s before an active
   * shift. Each "last 2s of shift N" window is always true because either that shift is active for
   * us, or it's the pre-spinup for the next shift which is.
   */
  public static boolean shouldSpinupFlywheel(double matchTime) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return false;
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) return false;

    boolean redInactiveFirst =
        switch (gameData.charAt(0)) {
          case 'R' -> true;
          case 'B' -> false;
          default -> {
            yield true;
          }
        };

    boolean shift1ActiveForUs =
        alliance.get() == Alliance.Red ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > 130) return true; // transition
    if (matchTime > 107) return shift1ActiveForUs; // shift 1 (excl. pre-spinup window)
    if (matchTime > 105) return true; // last 2s shift 1 / pre-spinup shift 2
    if (matchTime > 82) return !shift1ActiveForUs; // shift 2 (excl. pre-spinup window)
    if (matchTime > 80) return true; // last 2s shift 2 / pre-spinup shift 3
    if (matchTime > 57) return shift1ActiveForUs; // shift 3 (excl. pre-spinup window)
    if (matchTime > 55) return true; // last 2s shift 3 / pre-spinup shift 4
    if (matchTime > 30) return !shift1ActiveForUs; // shift 4
    return true; // end game
  }

  public void teleopInit() {
    superstructure.setWantedSuperState(WantedState.INTAKING);
    // Reset defense mode at the start of each teleop period
    if (defenseModeActive) {
      defenseModeActive = false;
      drive.setDefenseMode(false);
      shooter.setDefenseMode(false);
      hopper.setDefenseMode(false);
      intakeRoller.setDefenseMode(false);
      intakePivot.setDefenseMode(false);
    }
  }
}
