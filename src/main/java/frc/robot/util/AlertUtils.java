package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;

public class AlertUtils {
  public static void updateAlert(Alert alert, boolean condition) {
    if (condition) {
      alert.set(true);
    } else {
      alert.set(false);
    }
  }
}
