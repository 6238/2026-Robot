package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class IntakeIOTalonFX implements IntakeIO {
  // Alerts
  private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
  private Alert intakeArmConfigAlert =
      new Alert("CRITICAL", "Failed to Configure Intake Arm Motor", AlertType.kError);
  private Alert intakeConfigAlert =
      new Alert("CRITICAL", "Failed To Configure Intake Motor", AlertType.kError);

  // Motor Controllers
  public TalonFX intakeTalon;
  public TalonFX intakeArmTalon;

  // Status Signals
  public BetterStatusSignalCollection statusSignalCollector;

  public StatusSignal<AngularVelocity> intakeVelocity;
  public StatusSignal<AngularAcceleration> intakeAcceleration;
  public StatusSignal<Current> intakeCurrent;
  public StatusSignal<Voltage> intakeVoltage;
  public StatusSignal<Temperature> intakeTemperature;

  public StatusSignal<Angle> intakeArmPosition;
  public StatusSignal<AngularVelocity> intakeArmVelocity;
  public StatusSignal<AngularAcceleration> intakeArmAcceleration;
  public StatusSignal<Current> intakeArmCurrent;
  public StatusSignal<Voltage> intakeArmVoltage;

  // Control Requests
  public MotionMagicVoltage motionMagicVoltageArm = new MotionMagicVoltage(0).withSlot(0);
  public VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public IntakeIOTalonFX() {
    this.intakeTalon = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.CAN_BUS);
    this.intakeArmTalon = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_ID, IntakeConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;
    intakeConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_DIRECTION;
    intakeConfig.Slot0 = IntakeConstants.INTAKE_GAINS.toSlot0Configs();

    AlertUtils.processCriticalAlert(
        intakeConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeTalon.getConfigurator().apply(intakeConfig)));

    TalonFXConfiguration intakeArmConfig = new TalonFXConfiguration();
    intakeArmConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_ARM_GEARING;
    intakeArmConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_ARM_MOTOR_DIRECTION;
    intakeArmConfig.Slot0 = IntakeConstants.INTAKE_ARM_GAINS.toSlot0Configs();
    intakeArmConfig.MotionMagic = IntakeConstants.INTAKE_ARM_MOTION_MAGIC_CONFIGS;
    intakeArmConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    intakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    AlertUtils.processCriticalAlert(
        intakeArmConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeArmTalon.getConfigurator().apply(intakeArmConfig)));

    intakeArmTalon.setPosition(IntakeConstants.INTAKE_START_VALUE);

    // Status Signals
    intakeVelocity = intakeTalon.getVelocity();
    intakeAcceleration = intakeTalon.getAcceleration();
    intakeCurrent = intakeTalon.getStatorCurrent();
    intakeVoltage = intakeTalon.getMotorVoltage();
    intakeTemperature = intakeTalon.getDeviceTemp();

    intakeArmPosition = intakeArmTalon.getPosition();
    intakeArmVelocity = intakeArmTalon.getVelocity();
    intakeArmAcceleration = intakeArmTalon.getAcceleration();
    intakeArmCurrent = intakeArmTalon.getStatorCurrent();
    intakeArmVoltage = intakeArmTalon.getMotorVoltage();

    statusSignalCollector =
        new BetterStatusSignalCollection(
            intakeVelocity,
            intakeAcceleration,
            intakeCurrent,
            intakeVoltage,
            intakeTemperature,
            intakeArmPosition,
            intakeArmVelocity,
            intakeArmAcceleration,
            intakeArmCurrent,
            intakeArmVoltage);
    statusSignalCollector.setUpdateFrequencyForAll(50);
    ParentDevice.optimizeBusUtilizationForAll(intakeTalon, intakeArmTalon);
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
    inputs.intakeArmTalonConnected = intakeArmTalon.isConnected();

    inputs.intakeVelocity = intakeVelocity.getValue();
    inputs.intakeAcceleration = intakeAcceleration.getValue();
    inputs.intakeAppliedCurrent = intakeCurrent.getValue();
    inputs.intakeAppliedVoltage = intakeVoltage.getValue();
    inputs.intakeTemperature = intakeTemperature.getValue();

    inputs.intakeArmPosition = intakeArmPosition.getValue();
    inputs.intakeArmVelocity = intakeArmVelocity.getValue();
    inputs.intakeArmAcceleration = intakeArmAcceleration.getValue();
    inputs.intakeArmAppliedCurrent = intakeArmCurrent.getValue();
    inputs.intakeArmAppliedVoltage = intakeArmVoltage.getValue();
  }

  public void setIntakeVoltage(Voltage voltage) {
    intakeTalon.setVoltage(voltage.in(Volts));
  }

  public void setIntakeVelocity(AngularVelocity speed) {
    intakeTalon.setControl(velocityVoltage.withVelocity(speed));
  }

  public void setIntakeArmVoltage(Voltage volts) {
    intakeArmTalon.setVoltage(volts.in(Volts));
  }

  public void setIntakePosition(Angle targetAngle) {
    intakeArmTalon.setControl(motionMagicVoltageArm.withPosition(targetAngle));
  }

  public void resetArmAngle() {
    intakeArmTalon.setPosition(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
  }
}
