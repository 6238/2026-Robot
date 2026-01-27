package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.AlertUtils;
import frc.robot.util.BetterStatusSignalCollection;

public class IntakeIOTalonFX implements IntakeIO {
  // Alerts
  private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
  private Alert intakeConfigAlert =
      new Alert("CRITICAL", "Failed To Configure Intake Motor", AlertType.kError);

  // Motor Controllers
  public TalonFX intakeTalon;

  // Status Signals
  public BetterStatusSignalCollection statusSignalCollector;

  public StatusSignal<AngularVelocity> intakeVelocity;
  public StatusSignal<AngularAcceleration> intakeAcceleration;
  public StatusSignal<Current> intakeCurrent;
  public StatusSignal<Voltage> intakeVoltage;

  public IntakeIOTalonFX() {
    this.intakeTalon = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;
    intakeConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_DIRECTION;

    AlertUtils.processCriticalAlert(
        intakeConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeTalon.getConfigurator().apply(intakeConfig)));

    // Status Signals
    intakeVelocity = intakeTalon.getVelocity();
    intakeAcceleration = intakeTalon.getAcceleration();
    intakeCurrent = intakeTalon.getStatorCurrent();
    intakeVoltage = intakeTalon.getMotorVoltage();

    statusSignalCollector =
        new BetterStatusSignalCollection(
            intakeVelocity, intakeAcceleration, intakeCurrent, intakeVoltage);
    statusSignalCollector.setUpdateFrequencyForAll(50);
    ParentDevice.optimizeBusUtilizationForAll(intakeTalon);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    statusSignalCollector.refreshAll();

    if (!statusSignalCollector.isAllGood()) {
      statusSignalAlert.set(true);
      statusSignalAlert.setText(
          "Indexer TalonFX Status Signal Error: " + statusSignalCollector.getBadSignalsString());
    } else {
      statusSignalAlert.set(false);
    }

    inputs.intakeTalonConnected = intakeTalon.isConnected();

    inputs.intakeVelocity = intakeVelocity.getValue();
    inputs.intakeAcceleration = intakeAcceleration.getValue();
    inputs.intakeAppliedCurrent = intakeCurrent.getValue();
    inputs.intakeAppliedVoltage = intakeVoltage.getValue();
  }

  public void setIntakeVoltage(Voltage voltage) {
    intakeTalon.setVoltage(voltage.in(Volts));
  }
}
