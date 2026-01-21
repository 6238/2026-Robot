package frc.robot.sim;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.MapleSimSwerve;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSetpoint;
import frc.robot.util.RobotIdentity;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class AIOpponentRobotSim extends SubsystemBase {
  // Off-field parking to keep physics cheaper when disabled. [page:1]
  public static final Pose2d[] QUEENING_POSES = {
    new Pose2d(-6, 0, new Rotation2d()),
    new Pose2d(-5, 0, new Rotation2d()),
    new Pose2d(-4, 0, new Rotation2d()),
    new Pose2d(-3, 0, new Rotation2d()),
    new Pose2d(-2, 0, new Rotation2d())
  };

  // Put this in the neutral zone for your 2026 field coordinates.
  // (Use your field CAD/measured coords here.)
  private static final Pose2d NEUTRAL_WAIT_POSE = new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(180));

  private final int id;
  private final Pose2d queeningPose;

  private final SwerveDriveSimulation swerveDriveSimulation;
  private final SelfControlledSwerveDriveSimulation drive; // replacement for Simplified wrapper

  private final IntakeSimulation fuelIntake;

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
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
  }
  // PathPlanner bits (copied pattern from Maple-Sim opponent robot doc). [page:1]
  private static final double ROBOT_MASS_KG = 31.175;
  private static final double ROBOT_MOI = 2.15;
  private static final double WHEEL_COF = 1.2;
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

  private final PPHolonomicDriveController driveController =
      new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

  public AIOpponentRobotSim(int id) {
    this.id = id;
    this.queeningPose = QUEENING_POSES[id];

    // Create physics drivetrain sim and register it to the arena. [page:1]
    this.swerveDriveSimulation =
        MapleSimSwerve.createSimulationDrive(RobotIdentity.getTunerConstants(), queeningPose);

    // Wrap it with the 2026 self-controlled helper (API varies by version).
    this.drive = new SelfControlledSwerveDriveSimulation(swerveDriveSimulation);

    // Intake sim attached to the drivetrain; “touch it, get it” behavior. [page:0]
    this.fuelIntake =
        IntakeSimulation.InTheFrameIntake(
            "Fuel", // game piece type string
            swerveDriveSimulation, // drivetrain to attach to [page:0]
            Meters.of(1),
            IntakeSimulation.IntakeSide.FRONT,
            30);
  }

  private boolean shouldFlipPathForAlliance() {
    return DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue)
        .equals(DriverStation.Alliance.Red);
  }

  @Override
  public void periodic() {
    drive.addVisionEstimation(drive.getActualPoseInSimulationWorld(), Timer.getTimestamp());

    Logger.recordOutput("OpponentRobotPose" + this.id, this.drive.getActualPoseInSimulationWorld());
  }

  /** Follow one PathPlanner path (pattern from the Maple-Sim opponent robot example). [page:1] */
  private Command followPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        // Use *actual* sim pose/speeds (bypasses odometry noise), as in docs. [page:1]
        drive::getActualPoseInSimulationWorld,
        drive::getActualSpeedsRobotRelative,
        (speeds, feedforwards) -> drive.runChassisSpeeds(speeds, new Translation2d(), false, false),
        driveController,
        PP_CONFIG,
        this::shouldFlipPathForAlliance,
        this);
  }

  /** Move bot to neutral and stop. */
  public Command waitInNeutralZone() {
    return Commands.runOnce(() -> drive.setSimulationWorldPose(NEUTRAL_WAIT_POSE), this)
        .andThen(
            Commands.runOnce(
                () ->
                    drive.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false),
                this));
  }

  public Command intakeOn() {
    return Commands.runOnce(() -> fuelIntake.startIntake(), this); // startIntake pattern [page:0]
  }

  public Command intakeOff() {
    return Commands.runOnce(() -> fuelIntake.stopIntake(), this); // stopIntake pattern [page:0]
  }

  /** Build an auto-cycle: wait -> path while intaking -> shoot -> repeat. */
  public Command neutralCycle(PathPlannerPath driveOverFuelPath, Command shootCommand) {
    return Commands.sequence(
        runOnce(
            () -> drive.setSimulationWorldPose(driveOverFuelPath.getStartingHolonomicPose().get())),
        new SequentialCommandGroup(
                intakeOn(),
                followPath(driveOverFuelPath),
                intakeOff(),
                Commands.repeatingSequence(
                        Commands.sequence(shootCommand, Commands.waitSeconds(0.1)))
                    .withTimeout(3))
            .repeatedly());
  }

  /** Standard disable: park off-field and stop motion (pattern from docs). [page:1] */
  public Command disableCommand() {
    return Commands.runOnce(() -> drive.setSimulationWorldPose(queeningPose), this)
        .andThen(
            Commands.runOnce(
                () ->
                    drive.runChassisSpeeds(new ChassisSpeeds(), new Translation2d(), false, false),
                this))
        .ignoringDisable(true);
  }

  public Command shootFuelIntoAllianceArea(
      ShooterSetpoint shooterSetpoint /* your type */, double fixedHoodDeg /* your constant */) {

    return Commands.runOnce(
        () -> {
          // Remove 1 fuel from intake (pattern: obtainGamePieceFromIntake). [page:0]
          if (!fuelIntake.obtainGamePieceFromIntake()) return;

          var projectile =
              new RebuiltFuelOnFly(
                  swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                  new Translation2d(edu.wpi.first.math.util.Units.inchesToMeters(12), 0),
                  swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  Rotation2d.k180deg,
                  Inches.of(21),
                  MetersPerSecond.of(
                      shooterSetpoint.flywheelSpeed.in(RotationsPerSecond)
                          * 2
                          * 0.0508
                          * Math.PI
                          / 1.9),
                  ShooterConstants.FIXED_HOOD_ANGLE_DEGREES);

          // Register projectile into the arena (core projectile pattern). [page:2]
          SimulatedArena.getInstance().addGamePieceProjectile(projectile); // [page:2]
        },
        this);
  }

  public void buildChooser(PathPlannerPath fuelPath, Command shootCommand) {
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Disable", disableCommand()); // [page:1]
    chooser.addOption("Neutral Cycle", neutralCycle(fuelPath, shootCommand));
    chooser.onChange((Command::schedule));

    // Schedule the selected command when teleop starts
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> chooser.getSelected().schedule()));

    SmartDashboard.putData("AIRobotBehaviors/Opponent " + id, chooser); // [page:1]
  }
}
