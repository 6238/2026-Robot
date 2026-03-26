package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivot;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * DRS (Dynamic Recovery System) — Intake Lower Block.
 *
 * <p>Wraps an autonomous step so that if the intake pivot is blocked while lowering to the ground
 * setpoint, the robot automatically:
 *
 * <ol>
 *   <li><b>Detects</b> the block (handled in {@link IntakePivot#periodic()}).
 *   <li><b>Mitigates</b>: lifts to the middle position, drives backward briefly, then lowers again.
 *   <li><b>Resumes</b>: re-runs the original auto step from the current robot position.
 * </ol>
 */
public class DRSHelper {

  /**
   * Wraps an auto step with DRS monitoring.
   *
   * <p>The step is re-created fresh on each attempt (PathPlanner will re-plan from the robot's
   * current position). The wrapper exits as soon as the step completes without DRS firing.
   *
   * @param makeStep supplier that creates the auto step command (called once per attempt)
   * @param pivot the intake pivot subsystem
   * @param drive the drive subsystem
   * @param stepRequirements all subsystems that {@code makeStep} commands require — needed so the
   *     scheduler correctly allocates them for the deferred inner command
   */
  public static Command wrapWithDRS(
      Supplier<Command> makeStep, IntakePivot pivot, Drive drive, Set<Subsystem> stepRequirements) {

    boolean[] stepCompleted = {false};

    return Commands.sequence(
        Commands.runOnce(() -> stepCompleted[0] = false),
        Commands.defer(
                () ->
                    makeStep
                        .get()
                        .finallyDo(
                            interrupted -> {
                              if (!interrupted) stepCompleted[0] = true;
                            })
                        .until(pivot::isDRSTriggered)
                        .andThen(
                            Commands.either(
                                buildRecovery(pivot, drive),
                                Commands.none(),
                                () -> !stepCompleted[0])),
                stepRequirements)
            .repeatedly()
            .until(() -> stepCompleted[0]));
  }

  /**
   * Builds the three-phase DRS mitigation sequence: lift to middle → drive backward → lower to
   * ground.
   */
  private static Command buildRecovery(IntakePivot pivot, Drive drive) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("DRS/RecoveryActive", true);
                  pivot.setAngle(Degrees.of(IntakeConstants.DRS_MIDDLE_ANGLE_DEGREES.get()));
                }),
            Commands.waitUntil(
                () ->
                    Math.abs(
                            pivot.inputs.intakeArmPosition.in(Degrees)
                                - IntakeConstants.DRS_MIDDLE_ANGLE_DEGREES.get())
                        < IntakeConstants.DRS_MIDDLE_ANGLE_TOLERANCE_DEGREES),
            Commands.run(
                    () ->
                        drive.runVelocity(
                            new ChassisSpeeds(
                                -IntakeConstants.DRS_BACKUP_SPEED_MPS.get(), 0.0, 0.0)),
                    drive)
                .withTimeout(IntakeConstants.DRS_BACKUP_DURATION_SECONDS),
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
            Commands.runOnce(
                () -> pivot.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()))),
            Commands.waitUntil(
                () ->
                    Math.abs(
                            pivot.inputs.intakeArmPosition.in(Degrees)
                                - IntakeConstants.INTAKE_DOWN_VALUE.get())
                        < IntakeConstants.DRS_DOWN_ANGLE_TOLERANCE_DEGREES),
            Commands.runOnce(
                () -> {
                  pivot.resetDRS();
                  Logger.recordOutput("DRS/RecoveryActive", false);
                }))
        .withName("DRSRecovery");
  }

  private DRSHelper() {}
}
