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
import org.json.simple.parser.ParseException;

public class AutoRoutines {
  private final Drive drive;
  private final Superstructure superstructure;
  private final IntakePivot intakePivot;

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
        superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.IDLE));
  }

  public SendableChooser<Command> buildAutoChooser()
      throws FileVersionException, IOException, ParseException {
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    PathPlannerPath upperTrenchCycle1 = PathPlannerPath.fromPathFile("Upper Trench Cycle 1");
    PathPlannerPath upperTrenchCycle2 = PathPlannerPath.fromPathFile("Upper Trench Cycle 2");
    PathPlannerPath lowerTrenchCycle1 = PathPlannerPath.fromPathFile("Lower Trench Cycle 1");
    PathPlannerPath lowerTrenchCycle2 = PathPlannerPath.fromPathFile("Lower Trench Cycle 2");

    autoChooser.addOption(
        "Left Trench Mid Rush (double)",
        Commands.sequence(
            resetPoseCommand(upperTrenchCycle1),
            Commands.parallel(
                AutoBuilder.followPath(upperTrenchCycle1),
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(4.6),
            superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.INTAKING),
            AutoBuilder.followPath(upperTrenchCycle2),
            shootCommand(6)));
    autoChooser.addOption(
        "Right Trench Mid Rush (double)",
        Commands.sequence(
            intakePivot.preloadPivot(),
            resetPoseCommand(lowerTrenchCycle1),
            Commands.parallel(
                AutoBuilder.followPath(lowerTrenchCycle1),
                Commands.sequence(
                    Commands.waitSeconds(0.25),
                    superstructure.setWantedSuperStateCommand(
                        () -> Superstructure.WantedState.INTAKING))),
            shootCommand(4),
            superstructure.setWantedSuperStateCommand(() -> Superstructure.WantedState.INTAKING),
            AutoBuilder.followPath(lowerTrenchCycle2),
            shootCommand(6)));

    return autoChooser;
  }
}
