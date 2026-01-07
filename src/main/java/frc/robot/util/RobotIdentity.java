public class RobotIdentity {
    public Alert serialNumberAlert =
        new Alert(
            "Robot Serial Number Unknown",
            AlertType.kWarning,
            "RoboRIO serial number is unknown -> Defaulting to competition robot");

    public enum RobotType {
        COMP_BOT,
        PRACTICE_BOT,
    }

    public static String COMP_RIO_SERIAL = "R123456789"; // TODO: Replace with actual serial number of competition robot
    public static String PRACTICE_RIO_SERIAL = "R987654321"; // TODO: Replace with actual serial number of competition robot

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
            frc.robot.generated.comp.TunerConstants.DrivetrainConstants,
            frc.robot.generated.comp.TunerConstants.FrontLeft,
            frc.robot.generated.comp.TunerConstants.FrontRight,
            frc.robot.generated.comp.TunerConstants.BackLeft,
            frc.robot.generated.comp.TunerConstants.BackRight);
    }

    private static RobotTunerConstants fromPractice() {
        return new RobotTunerConstants(
            frc.robot.generated.practice.TunerConstants.DrivetrainConstants,
            frc.robot.generated.practice.TunerConstants.FrontLeft,
            frc.robot.generated.practice.TunerConstants.FrontRight,
            frc.robot.generated.practice.TunerConstants.BackLeft,
            frc.robot.generated.practice.TunerConstants.BackRight);
    }
}
