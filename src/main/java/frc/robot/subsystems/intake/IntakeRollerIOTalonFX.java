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
import org.littletonrobotics.junction.Logger;

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
  public StatusSignal<Current> intakeSupplyCurrent;
  public StatusSignal<Voltage> intakeVoltage;
  public StatusSignal<Temperature> intakeTemperature;

  private boolean notSwapped = true;

  public VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  public IntakeRollerIOTalonFX() {
    this.intakeTalon = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, IntakeConstants.ROLLER_CAN_BUS);
    this.intakeFollowerTalon =
        new TalonFX(IntakeConstants.INTAKE_FOLLOWER_MOTOR_ID, IntakeConstants.ROLLER_CAN_BUS);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_GEARING;
    intakeConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_DIRECTION;
    intakeConfig.Slot0 = IntakeConstants.INTAKE_GAINS.toSlot0Configs();
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 70;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 25;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    AlertUtils.processCriticalAlert(
        intakeConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeTalon.getConfigurator().apply(intakeConfig)));

    AlertUtils.processCriticalAlert(
        intakeConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> intakeFollowerTalon.getConfigurator().apply(intakeConfig)));

    intakeFollowerTalon.setControl(
        new Follower(IntakeConstants.INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));

    intakePosition = intakeTalon.getPosition();
    intakeVelocity = intakeTalon.getVelocity();
    intakeAcceleration = intakeTalon.getAcceleration();
    intakeSupplyCurrent = intakeTalon.getSupplyCurrent();
    intakeVoltage = intakeTalon.getMotorVoltage();
    intakeTemperature = intakeTalon.getDeviceTemp();

    statusSignalCollector =
        new BetterStatusSignalCollection(
            intakePosition,
            intakeVelocity,
            intakeAcceleration,
            intakeSupplyCurrent,
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
    inputs.intakeSupplyCurrent = intakeSupplyCurrent.getValue();
    inputs.intakeAppliedVoltage = intakeVoltage.getValue();
    inputs.intakeTemperature = intakeTemperature.getValue();

    inputs.intakeFollowerTalonConnected = intakeFollowerTalon.isConnected();

    if (inputs.intakeFollowerTalonConnected && !inputs.intakeTalonConnected && notSwapped) {
      intakeTalon.setControl(
          new Follower(IntakeConstants.INTAKE_MOTOR_ID, MotorAlignmentValue.Opposed));
      notSwapped = false;
    }

    Logger.recordOutput("swapped leader follower", !notSwapped);
  }

  @Override
  public void setIntakeVoltage(Voltage voltage) {
    if (!notSwapped) {
      intakeFollowerTalon.setVoltage(-voltage.in(Volts));
    } else {
      intakeTalon.setVoltage(voltage.in(Volts));
    }
  }

  @Override
  public void setIntakeVelocity(AngularVelocity speed) {
    double currentVelocity = intakeVelocity.getValueAsDouble();
    double targetVelocity = speed.in(RotationsPerSecond);
    boolean recovering =
        currentVelocity < (targetVelocity - IntakeConstants.ROLLER_RECOVERY_THRESHOLD_RPS);
    Logger.recordOutput("intakerollerrecovering", recovering);
    if (!notSwapped) {
      intakeFollowerTalon.setControl(
          velocityVoltage
              .withVelocity(speed.times(-1))
              .withAcceleration(
                  recovering ? -IntakeConstants.ROLLER_RECOVERY_ACCELERATION_RPS2 : 0));
    } else {
      intakeTalon.setControl(
          velocityVoltage
              .withVelocity(speed)
              .withAcceleration(
                  recovering ? IntakeConstants.ROLLER_RECOVERY_ACCELERATION_RPS2 : 0));
    }
  }
}
