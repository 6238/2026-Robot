package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;

public class AlertUtils {
  public static Runnable criticalErrorRumbleFunction;
  public static Runnable stopRumbleFunction;

  public static void processAlert(Alert alert, boolean condition) {
    if (condition) {
      alert.set(true);
    } else {
      alert.set(false);
    }
  }

  public static void clearCriticalAlerts() {
    stopRumbleFunction.run();
  }

  public static void processCriticalAlert(Alert alert, boolean condition) {
    if (condition) {
      alert.set(true);
      criticalErrorRumbleFunction.run();
    } else {
      alert.set(false);
    }
  }
}
