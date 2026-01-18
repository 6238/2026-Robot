package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.COMPTunerConstants;
import frc.robot.generated.PRACTICETunerConstants;
import frc.robot.generated.RobotTunerConstants;
import org.littletonrobotics.junction.Logger;

public class RobotIdentity {
  public static Alert serialNumberAlert =
      new Alert(
          "RobotIdentity",
          "Serial number is unknown -> Defaulting to competition robot",
          AlertType.kWarning);

  public enum RobotType {
    COMP_BOT,
    PRACTICE_BOT,
  }

  public static final String COMP_RIO_SERIAL = "032B1F73";
  public static final String PRACTICE_RIO_SERIAL = "03182373";

  public static RobotType getRobotType() {
    switch (RobotController.getSerialNumber()) {
      case COMP_RIO_SERIAL:
        return RobotType.COMP_BOT;
      case PRACTICE_RIO_SERIAL:
        return RobotType.PRACTICE_BOT;
      default:
        serialNumberAlert.set(true);
        return RobotType.COMP_BOT;
    }
  }

  // Cache so it only decides once
  private static RobotTunerConstants cached;

  public static RobotTunerConstants getTunerConstants() {
    if (cached != null) return cached;

    if (RobotBase.isSimulation()) {
      cached = fromComp();
      return cached;
    }

    Logger.recordMetadata("Robot Type", getRobotType().toString());

    switch (getRobotType()) {
      case COMP_BOT:
        cached = fromComp();
        break;
      case PRACTICE_BOT:
        cached = fromPractice();
        break;
    }

    return cached;
  }

  private static RobotTunerConstants fromComp() {
    return new RobotTunerConstants(
        COMPTunerConstants.DrivetrainConstants,
        COMPTunerConstants.FrontLeft,
        COMPTunerConstants.FrontRight,
        COMPTunerConstants.BackLeft,
        COMPTunerConstants.BackRight,
        COMPTunerConstants.kSpeedAt12Volts);
  }

  private static RobotTunerConstants fromPractice() {
    return new RobotTunerConstants(
        PRACTICETunerConstants.DrivetrainConstants,
        PRACTICETunerConstants.FrontLeft,
        PRACTICETunerConstants.FrontRight,
        PRACTICETunerConstants.BackLeft,
        PRACTICETunerConstants.BackRight,
        PRACTICETunerConstants.kSpeedAt12Volts);
  }
}
