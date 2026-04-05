package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleCommand extends Command {
  private final Command m_onCommand;
  private final Command m_offCommand;
  private boolean m_isOn = false;

  /**
   * Creates a new ToggleCommand.
   *
   * @param onCommand The command to run on the first/odd presses.
   * @param offCommand The command to run on the second/even presses.
   */
  public ToggleCommand(Command onCommand, Command offCommand) {
    m_onCommand = onCommand;
    m_offCommand = offCommand;

    // Requirements: Usually, you don't add requirements here because
    // this is a "meta-command" that delegates to others.
  }

  @Override
  public void initialize() {
    if (m_isOn) {
      m_offCommand.schedule();
    } else {
      m_onCommand.schedule();
    }
    // Flip the state for the next press
    m_isOn = !m_isOn;
  }

  @Override
  public boolean isFinished() {
    // This command finishes instantly after scheduling the target command
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
