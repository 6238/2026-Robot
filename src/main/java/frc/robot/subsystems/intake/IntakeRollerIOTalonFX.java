package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.AlertUtils;
import frc.robot.util.BetterStatusSignalCollection;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
  private Alert intakeConfigAlert =
      new Alert("CRITICAL", "Failed To Configure Intake Motor", AlertType.kError);

  public TalonFX intakeTalon;
  public TalonFX intakeFollowerTalon;

  public BetterStatusSignalCollection statusSignalCollector;

  public StatusSignal<Angle> intakePosition;
  public StatusSignal<AngularVelocity> intakeVelocity;
  public StatusSignal<AngularAcceleration> intakeAcceleration;
  public StatusSignal<Current> intakeCurrent;
  public StatusSignal<Voltage> intakeVoltage;
  public StatusSignal<Temperature> intakeTemperature;

  public VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public IntakeRollerIOTalonFX() {
    this.intakeTalon = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS);
    this.intakeFollowerTalon =
        new TalonFX(IntakeConstants.INTAKE_FOLLOWER_MOTOR_ID, IntakeConstants.CAN_BUS);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;
    intakeConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_DIRECTION;
    intakeConfig.Slot0 = IntakeConstants.INTAKE_GAINS.toSlot0Configs();

    AlertUtils.processCriticalAlert(
        intakeConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeTalon.getConfigurator().apply(intakeConfig)));

    intakeFollowerTalon.setControl(
        new Follower(IntakeConstants.INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));

    intakePosition = intakeTalon.getPosition();
    intakeVelocity = intakeTalon.getVelocity();
    intakeAcceleration = intakeTalon.getAcceleration();
    intakeCurrent = intakeTalon.getStatorCurrent();
    intakeVoltage = intakeTalon.getMotorVoltage();
    intakeTemperature = intakeTalon.getDeviceTemp();

    statusSignalCollector =
        new BetterStatusSignalCollection(
            intakePosition,
            intakeVelocity,
            intakeAcceleration,
            intakeCurrent,
            intakeVoltage,
            intakeTemperature);
    statusSignalCollector.setUpdateFrequencyForAll(50);
    ParentDevice.optimizeBusUtilizationForAll(intakeTalon, intakeFollowerTalon);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    statusSignalCollector.refreshAll();

    if (!statusSignalCollector.isAllGood()) {
      statusSignalAlert.set(true);
      statusSignalAlert.setText(
          "Intake Roller TalonFX Status Signal Error: "
              + statusSignalCollector.getBadSignalsString());
    } else {
      statusSignalAlert.set(false);
    }

    inputs.intakeTalonConnected = intakeTalon.isConnected();
    inputs.intakePosition = intakePosition.getValue();
    inputs.intakeVelocity = intakeVelocity.getValue();
    inputs.intakeAcceleration = intakeAcceleration.getValue();
    inputs.intakeAppliedCurrent = intakeCurrent.getValue();
    inputs.intakeAppliedVoltage = intakeVoltage.getValue();
    inputs.intakeTemperature = intakeTemperature.getValue();
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    intakeTalon.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setIntakeVelocity(AngularVelocity speed) {
    intakeTalon.setControl(velocityVoltage.withVelocity(speed));
  }
}
