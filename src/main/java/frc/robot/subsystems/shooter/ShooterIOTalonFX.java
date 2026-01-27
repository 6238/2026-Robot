package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private TalonFX feederTalon;

  // Status Signal Management
  public BetterStatusSignalCollection statusSignalCollector;

  // flywheel Inputs
  public StatusSignal<AngularVelocity> flywheelVelocity;
  public StatusSignal<AngularAcceleration> flywheelAcceleration;
  public StatusSignal<Current> flywheelAppliedCurent;
  public StatusSignal<Voltage> flywheelAppliedVoltage;

  // Feeder Inputs
  public StatusSignal<AngularVelocity> feederVelocity;
  public StatusSignal<AngularAcceleration> feederAcceleration;
  public StatusSignal<Current> feederAppliedCurent;
  public StatusSignal<Voltage> feederAppliedVoltage;

  // Control Requests
  public VelocityVoltage flywheelVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  public ShooterIOTalonFX() {
    flywheelTalon = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEARING;
    flywheelConfig.Slot0 = ShooterConstants.FLYWHEEL_GAINS.toSlot0Configs();
    flywheelConfig.MotionMagic = ShooterConstants.FLYWHEEL_MOTION_MAGIC_CONFIGS;
    flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_INVERTED;

    TalonFX secondFlywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL2_MOTOR_ID);
    secondFlywheelMotor.setControl(
        new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FEEDER_GEARING;
    feederConfig.MotorOutput.Inverted = ShooterConstants.FEEDER_INVERTED;

    AlertUtils.processCriticalAlert(flywheelConfigAlert, !tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES,
        () -> flywheelTalon.getConfigurator().apply(flywheelConfig)));
    
    AlertUtils.processCriticalAlert(feederConfigAlert, !tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES,
        () -> feederTalon.getConfigurator().apply(feederConfig)));

    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelAcceleration = flywheelTalon.getAcceleration();
    flywheelAppliedCurent = flywheelTalon.getStatorCurrent();
    flywheelAppliedVoltage = flywheelTalon.getMotorVoltage();

    feederVelocity = feederTalon.getVelocity();
    feederAcceleration = feederTalon.getAcceleration();
    feederAppliedCurent = feederTalon.getStatorCurrent();
    feederAppliedVoltage = feederTalon.getMotorVoltage();

    // Initialize Status Signals
    statusSignalCollector =
        new BetterStatusSignalCollection(
            flywheelVelocity,
            flywheelAcceleration,
            flywheelAppliedCurent,
            flywheelAppliedVoltage,
            feederVelocity,
            feederAcceleration,
            feederAppliedCurent,
            feederAppliedVoltage);
    statusSignalCollector.setUpdateFrequencyForAll(50);
    ParentDevice.optimizeBusUtilizationForAll(flywheelTalon, feederTalon);
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
    inputs.flywheelAppliedCurrent = flywheelAppliedCurent.getValue();
    inputs.flywheelAppliedVoltage = flywheelAppliedVoltage.getValue();

    inputs.feederVelocity = feederVelocity.getValue();
    inputs.feederAcceleration = feederAcceleration.getValue();
    inputs.feederAppliedCurrent = feederAppliedCurent.getValue();
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
}
