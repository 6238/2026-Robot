package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Superstructure.CurrentState;
import java.util.function.Supplier;

public class GratuitousLighting extends SubsystemBase {
  private final CANBus candleCanBus = new CANBus("canivore");
  private final CANdle candle;

  /** Set by RobotContainer to reflect the current superstructure state. */
  public Supplier<CurrentState> superState = () -> CurrentState.IDLE;

  // All colors/brightness kept at ~20% to avoid blinding drive teams.
  private static final EmptyAnimation idleAnimation = new EmptyAnimation(0);

  private static final StrobeAnimation intakingAnimation =
      new StrobeAnimation(0, 140).withColor(new RGBWColor(0, 50, 0)).withFrameRate(4);

  private static final StrobeAnimation shootingAnimation =
      new StrobeAnimation(0, 140).withColor(new RGBWColor(50, 20, 0)).withFrameRate(8);

  private static final StrobeAnimation shiftSoonAnimation =
      new StrobeAnimation(0, 140).withColor(new RGBWColor(0, 0, 50)).withFrameRate(6);

  public GratuitousLighting() {
    candle = new CANdle(45, candleCanBus);
  }

  /** Returns true during the 2-second window before each shift transition in teleop. */
  private boolean isShiftAboutToStart() {
    if (!DriverStation.isTeleopEnabled()) return false;
    double t = DriverStation.getMatchTime();
    return (t >= 105 && t <= 107)
        || (t >= 80 && t <= 82)
        || (t >= 55 && t <= 57)
        || (t >= 30 && t <= 32);
  }

  @Override
  public void periodic() {

    if (!DriverStation.isEnabled()) {
      candle.setControl(idleAnimation);
      return;
    }

    if (isShiftAboutToStart()) {
      candle.setControl(shiftSoonAnimation);
      return;
    }

    CurrentState state = superState.get();
    if (state == CurrentState.SHOOTING
        || state == CurrentState.SPINNING_UP
        || state == CurrentState.PASSING) {
      candle.setControl(shootingAnimation);
    } else if (state == CurrentState.INTAKING) {
      candle.setControl(intakingAnimation);
    } else {
      candle.setControl(idleAnimation);
    }
  }
}
