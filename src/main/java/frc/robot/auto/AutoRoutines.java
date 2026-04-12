package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.superstructure.Superstructure;
import java.io.IOException;
import java.util.Set;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class AutoRoutines {
  private final Drive drive;
  private final Superstructure superstructure;
  private final IntakePivot intakePivot;
  private final LoggedNetworkNumber autoDelay = new LoggedNetworkNumber("Auto Delay");

  public AutoRoutines(Drive drive, Superstructure superstructure, IntakePivot intakePivot) {
    this.drive = drive;
    this.superstructure = superstructure;
    this.intakePivot = intakePivot;
  }

  private Command resetPoseCommand(PathPlannerPath path) {
    return path.getStartingHolonomicPose().map(AutoBuilder::resetOdom).orElse(Commands.none());
  }

  private Command shootCommand(double time) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(time),
            superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.SHOOTING),
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0,
                () -> superstructure.getShotSetpoint().robotPose.getRotation())),
        superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.INTAKING));
  }

  private Command passCommand(double time) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(time),
            superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.PASSING),
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> 0,
                () -> 0,
                () -> superstructure.getShotSetpoint().robotPose.getRotation())),
        superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.INTAKING));
  }

  public SendableChooser<Command> buildAutoChooser()
      throws FileVersionException, IOException, ParseException {
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    PathPlannerPath lowerTrenchCycle1 = PathPlannerPath.fromPathFile("Lower Trench Cycle 1");
    PathPlannerPath lowerTrenchCycle2 = PathPlannerPath.fromPathFile("Lower Trench Cycle 2");
    PathPlannerPath lowerTrenchCycle3 = PathPlannerPath.fromPathFile("Lower Trench Cycle 3");

    autoChooser.addOption(
        "Right Trench Mid Rush (double)",
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(autoDelay.get()), Set.of()),
            resetPoseCommand(lowerTrenchCycle1),
            Commands.parallel(
                AutoBuilder.followPath(lowerTrenchCycle1),
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(9),
            //AutoBuilder.followPath(lowerTrenchCycle2),
            //shootCommand(4),
            AutoBuilder.followPath(lowerTrenchCycle3)));

    PathPlannerPath lowerTrenchDelayMidPath =
        PathPlannerPath.fromPathFile("Lower Trench Delay Path");
    PathPlannerPath lowerRiskyTrenchDelayMidPath =
        PathPlannerPath.fromPathFile("Lower Risky Trench Delay Path");
    PathPlannerPath lowerTrenchDelayToDepot =
        PathPlannerPath.fromPathFile("Lower Trench Delay to Depot");

    autoChooser.addOption(
        "Left Trench Delay Mid -> Depot",
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(autoDelay.get()), Set.of()),
            resetPoseCommand(lowerTrenchDelayMidPath),
            Commands.parallel(
                AutoBuilder.followPath(lowerTrenchDelayMidPath),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(4.5),
            AutoBuilder.followPath(lowerTrenchDelayToDepot),
            shootCommand(4.5)));

    autoChooser.addOption(
        "Left Risky Trench Delay Mid -> Depot",
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(autoDelay.get()), Set.of()),
            resetPoseCommand(lowerRiskyTrenchDelayMidPath),
            Commands.parallel(
                AutoBuilder.followPath(lowerRiskyTrenchDelayMidPath),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(4.5),
            AutoBuilder.followPath(lowerTrenchDelayToDepot),
            shootCommand(4.5)));

    PathPlannerPath upperTrenchCycle1 = PathPlannerPath.fromPathFile("Upper Trench Cycle 1");
    PathPlannerPath upperTrenchCycle2 = PathPlannerPath.fromPathFile("Upper Trench Cycle 2");
    PathPlannerPath upperTrenchCycle3 = PathPlannerPath.fromPathFile("Upper Trench Cycle 3");

    autoChooser.addOption(
        "Left Trench Mid Rush (double)",
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(autoDelay.get()), Set.of()),
            resetPoseCommand(upperTrenchCycle1),
            Commands.parallel(
                AutoBuilder.followPath(upperTrenchCycle1),
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(3.5),
            AutoBuilder.followPath(upperTrenchCycle2),
            shootCommand(4),
            AutoBuilder.followPath(upperTrenchCycle3)));

    PathPlannerPath upperTrenchDelayMidPath =
        PathPlannerPath.fromPathFile("Upper Trench Delay Path");
    PathPlannerPath upperRiskyTrenchDelayMidPath =
        PathPlannerPath.fromPathFile("Upper Risky Trench Delay Path");

    autoChooser.addOption(
        "Right Trench Delay Mid",
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(autoDelay.get()), Set.of()),
            resetPoseCommand(upperTrenchDelayMidPath),
            Commands.parallel(
                AutoBuilder.followPath(upperTrenchDelayMidPath),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(10)));

    autoChooser.addOption(
        "Right Risky Trench Delay Mid",
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(autoDelay.get()), Set.of()),
            resetPoseCommand(upperRiskyTrenchDelayMidPath),
            Commands.parallel(
                AutoBuilder.followPath(upperRiskyTrenchDelayMidPath),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(10)));

    PathPlannerPath trenchToMid = PathPlannerPath.fromPathFile("Trench to mid");
    PathPlannerPath midintake1 = PathPlannerPath.fromPathFile("Mid intake");
    PathPlannerPath midintake2 = PathPlannerPath.fromPathFile("Mid intake part 2");
    PathPlannerPath midintake3 = PathPlannerPath.fromPathFile("Mid intake part 3");

    autoChooser.addOption(
        "Elliot Special",
        Commands.sequence(
            resetPoseCommand(trenchToMid),
            Commands.parallel(
                AutoBuilder.followPath(trenchToMid),
                Commands.sequence(
                    Commands.waitSeconds(0.2),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            AutoBuilder.followPath(midintake1),
            passCommand(4.5),
            AutoBuilder.followPath(midintake2),
            passCommand(4.5),
            AutoBuilder.followPath(midintake3),
            passCommand(4.5)));

    PathPlannerPath depotPath = PathPlannerPath.fromPathFile("Depot");

    autoChooser.addOption(
        "Depot",
        Commands.sequence(
            Commands.waitSeconds(autoDelay.get()),
            resetPoseCommand(depotPath),
            Commands.parallel(
                AutoBuilder.followPath(depotPath),
                Commands.sequence(
                    Commands.waitSeconds(0.1),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(10)));

    return autoChooser;
  }
}
