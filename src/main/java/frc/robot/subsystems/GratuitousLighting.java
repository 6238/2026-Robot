package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
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

  /** Set after test-mode completes: true = all pass, false = any failure. Null while running. */
  private Boolean testResult = null;

  public void setTestResult(Boolean passed) {
    testResult = passed;
  }

  // All colors/brightness kept at ~20% to avoid blinding drive teams.
  private static final EmptyAnimation idleAnimation = new EmptyAnimation(0);

  private static final SolidColor testPassAnimation =
      new SolidColor(0, 68).withColor(new RGBWColor(0, 50, 0));
  private static final SolidColor testFailAnimation =
      new SolidColor(0, 68).withColor(new RGBWColor(50, 0, 0));

  private static final SolidColor intakingAnimation =
      new SolidColor(0, 68).withColor(new RGBWColor(0, 50, 0));

  private static final LarsonAnimation shootingAnimation =
      new LarsonAnimation(8, 68).withColor(new RGBWColor(100, 0, 0)).withFrameRate(50).withSize(10);

  private static final SolidColor shiftSoonAnimation =
      new SolidColor(0, 68).withColor(new RGBWColor(0, 0, 100));

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

    if (testResult != null) {
      candle.setControl(testResult ? testPassAnimation : testFailAnimation);
      return;
    }

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
