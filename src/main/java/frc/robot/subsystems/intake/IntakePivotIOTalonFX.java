package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.AlertUtils;
import frc.robot.util.BetterStatusSignalCollection;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
  private Alert intakeArmConfigAlert =
      new Alert("CRITICAL", "Failed to Configure Intake Arm Motor", AlertType.kError);

  public TalonFX intakeArmTalon;

  public BetterStatusSignalCollection statusSignalCollector;

  public StatusSignal<Angle> intakeArmPosition;
  public StatusSignal<AngularVelocity> intakeArmVelocity;
  public StatusSignal<AngularAcceleration> intakeArmAcceleration;
  public StatusSignal<Current> intakeArmSupplyCurrent;
  public StatusSignal<Voltage> intakeArmVoltage;

  public MotionMagicVoltage motionMagicVoltageArm = new MotionMagicVoltage(0).withSlot(0);

  public IntakePivotIOTalonFX() {
    this.intakeArmTalon = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_ID, IntakeConstants.CAN_BUS);

    TalonFXConfiguration intakeArmConfig = new TalonFXConfiguration();
    intakeArmConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_ARM_GEARING;
    intakeArmConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_ARM_MOTOR_DIRECTION;
    intakeArmConfig.Slot0 = IntakeConstants.INTAKE_ARM_GAINS.toSlot0Configs();
    intakeArmConfig.MotionMagic = IntakeConstants.INTAKE_ARM_MOTION_MAGIC_CONFIGS;
    intakeArmConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    intakeArmConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeArmConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeArmConfig.CurrentLimits.StatorCurrentLimit = 40;
    intakeArmConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeArmConfig.CurrentLimits.SupplyCurrentLimit = 30;
    intakeArmConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
    intakeArmConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

    AlertUtils.processCriticalAlert(
        intakeArmConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeArmTalon.getConfigurator().apply(intakeArmConfig)));

    intakeArmTalon.setPosition(IntakeConstants.INTAKE_START_VALUE);

    intakeArmPosition = intakeArmTalon.getPosition();
    intakeArmVelocity = intakeArmTalon.getVelocity();
    intakeArmAcceleration = intakeArmTalon.getAcceleration();
    intakeArmSupplyCurrent = intakeArmTalon.getSupplyCurrent();
    intakeArmVoltage = intakeArmTalon.getMotorVoltage();

    statusSignalCollector =
        new BetterStatusSignalCollection(
            intakeArmPosition,
            intakeArmVelocity,
            intakeArmAcceleration,
            intakeArmSupplyCurrent,
            intakeArmVoltage);
    statusSignalCollector.setUpdateFrequencyForAll(50);
    ParentDevice.optimizeBusUtilizationForAll(intakeArmTalon);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    statusSignalCollector.refreshAll();

    if (!statusSignalCollector.isAllGood()) {
      statusSignalAlert.set(true);
      statusSignalAlert.setText(
          "Intake Pivot TalonFX Status Signal Error: "
              + statusSignalCollector.getBadSignalsString());
    } else {
      statusSignalAlert.set(false);
    }

    inputs.intakeArmTalonConnected = intakeArmTalon.isConnected();
    inputs.intakeArmPosition = intakeArmPosition.getValue();
    inputs.intakeArmVelocity = intakeArmVelocity.getValue();
    inputs.intakeArmAcceleration = intakeArmAcceleration.getValue();
    inputs.intakeArmSupplyCurrent = intakeArmSupplyCurrent.getValue();
    inputs.intakeArmAppliedVoltage = intakeArmVoltage.getValue();
  }

  @Override
  public void setIntakePosition(Angle targetAngle) {
    intakeArmTalon.setControl(motionMagicVoltageArm.withPosition(targetAngle));
  }

  @Override
  public void setIntakeArmVoltage(Voltage volts) {
    intakeArmTalon.setVoltage(volts.in(Volts));
  }

  @Override
  public void resetArmAngle() {
    intakeArmTalon.setPosition(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    intakeArmTalon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
