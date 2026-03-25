package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.AlertUtils;
import frc.robot.util.BetterStatusSignalCollection;

public class ShooterIOTalonFX implements ShooterIO {
  // Alerts
  private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
  private Alert flywheelConfigAlert =
      new Alert("CRITICAL", "Failed To Configure Flywheel Motor", AlertType.kError);
  private Alert feederConfigAlert =
      new Alert("CRITICAL", "Failed To Configure Feeder Motor", AlertType.kError);

  // Motor Controllers
  private TalonFX flywheelTalon;
  private TalonFX flywheel2Talon;
  private TalonFX feederTalon;

  // Status Signal Management
  public BetterStatusSignalCollection statusSignalCollector;

  // flywheel Inputs
  public StatusSignal<AngularVelocity> flywheelVelocity;
  public StatusSignal<AngularAcceleration> flywheelAcceleration;
  public StatusSignal<Current> flywheelSupplyCurent;
  public StatusSignal<Current> flywheel2SupplyCurent;
  public StatusSignal<Voltage> flywheelAppliedVoltage;

  // Feeder Inputs
  public StatusSignal<AngularVelocity> feederVelocity;
  public StatusSignal<AngularAcceleration> feederAcceleration;
  public StatusSignal<Current> feederSupplyCurent;
  public StatusSignal<Voltage> feederAppliedVoltage;

  // Control Requests
  public MotionMagicVelocityVoltage flywheelVelocityVoltage =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  public MotionMagicVelocityVoltage feederVelocityVoltage =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  public ShooterIOTalonFX() {
    flywheelTalon = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEARING;
    flywheelConfig.Slot0 = ShooterConstants.FLYWHEEL_GAINS.toSlot0Configs();
    flywheelConfig.MotionMagic = ShooterConstants.FLYWHEEL_MOTION_MAGIC_CONFIGS;
    flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_INVERTED;

    flywheel2Talon = new TalonFX(ShooterConstants.FLYWHEEL2_MOTOR_ID, ShooterConstants.CAN_BUS);
    flywheel2Talon.setControl(
        new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FEEDER_GEARING;
    feederConfig.Slot0 = ShooterConstants.FEEDER_GAINS.toSlot0Configs();
    feederConfig.MotorOutput.Inverted = ShooterConstants.FEEDER_INVERTED;
    feederConfig.MotionMagic = ShooterConstants.FLYWHEEL_MOTION_MAGIC_CONFIGS;

    AlertUtils.processCriticalAlert(
        flywheelConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> flywheelTalon.getConfigurator().apply(flywheelConfig)));

    AlertUtils.processCriticalAlert(
        feederConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> feederTalon.getConfigurator().apply(feederConfig)));

    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelAcceleration = flywheelTalon.getAcceleration();
    flywheelSupplyCurent = flywheelTalon.getSupplyCurrent();
    flywheel2SupplyCurent = flywheel2Talon.getSupplyCurrent();
    flywheelAppliedVoltage = flywheelTalon.getMotorVoltage();

    feederVelocity = feederTalon.getVelocity();
    feederAcceleration = feederTalon.getAcceleration();
    feederSupplyCurent = feederTalon.getSupplyCurrent();
    feederAppliedVoltage = feederTalon.getMotorVoltage();

    // Initialize Status Signals
    statusSignalCollector =
        new BetterStatusSignalCollection(
            flywheelVelocity,
            flywheelAcceleration,
            flywheelSupplyCurent,
            flywheel2SupplyCurent,
            flywheelAppliedVoltage,
            feederVelocity,
            feederAcceleration,
            feederSupplyCurent,
            feederAppliedVoltage);
    statusSignalCollector.setUpdateFrequencyForAll(50);
    ParentDevice.optimizeBusUtilizationForAll(flywheelTalon, flywheel2Talon, feederTalon);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    statusSignalCollector.refreshAll();

    if (!statusSignalCollector.isAllGood()) {
      statusSignalAlert.set(true);
      statusSignalAlert.setText(
          "Shooter TalonFX Status Signal Error: " + statusSignalCollector.getBadSignalsString());
    } else {
      statusSignalAlert.set(false);
    }

    // Are motors connected?
    inputs.flywheelTalonConnected = flywheelTalon.isConnected();
    inputs.feederTalonConnected = feederTalon.isConnected();

    // Update Inputs
    inputs.flywheelVelocity = flywheelVelocity.getValue();
    inputs.flywheelAcceleration = flywheelAcceleration.getValue();
    inputs.flywheelSupplyCurrent = flywheelSupplyCurent.getValue();
    inputs.flywheel2SupplyCurrent = flywheel2SupplyCurent.getValue();
    inputs.flywheelAppliedVoltage = flywheelAppliedVoltage.getValue();

    inputs.feederVelocity = feederVelocity.getValue();
    inputs.feederAcceleration = feederAcceleration.getValue();
    inputs.feederSupplyCurrent = feederSupplyCurent.getValue();
    inputs.feederAppliedVoltage = feederAppliedVoltage.getValue();
  }

  public void setFlywheelSpeed(AngularVelocity speed) {
    flywheelTalon.setControl(flywheelVelocityVoltage.withVelocity(speed));
  }

  public void setFlywheelVoltage(Voltage voltage) {
    flywheelTalon.setVoltage(voltage.in(Volts));
  }

  public void setFeederVoltage(Voltage voltage) {
    feederTalon.setVoltage(voltage.in(Volts));
  }

  public void setFeederSpeed(AngularVelocity speed) {
    feederTalon.setControl(feederVelocityVoltage.withVelocity(speed));
  }
}
