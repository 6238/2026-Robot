package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.util.AlertUtils;
import frc.robot.util.BetterStatusSignalCollection;
import frc.robot.util.ThreadedBeamBreak;

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

  private final ThreadedBeamBreak beamBreak =
      new ThreadedBeamBreak(
          ShooterConstants.BEAM_BREAK_DIO_PORT, ShooterConstants.BEAM_BREAK_INVERTED);

  // Status Signal Management
  public BetterStatusSignalCollection statusSignalCollector;

  // flywheel Inputs
  public StatusSignal<AngularVelocity> flywheelVelocity;
  public StatusSignal<Current> flywheelSupplyCurent;
  public StatusSignal<Current> flywheel2SupplyCurent;
  public StatusSignal<Voltage> flywheelAppliedVoltage;

  // Feeder Inputs
  public StatusSignal<AngularVelocity> feederVelocity;
  public StatusSignal<Current> feederSupplyCurent;
  public StatusSignal<Voltage> feederAppliedVoltage;

  // Control Requests
  public MotionMagicVelocityVoltage flywheelVelocityVoltage =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  public MotionMagicVelocityVoltage feederVelocityVoltage =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  public VoltageOut flywheelVoltageOut = new VoltageOut(0);

  public ShooterIOTalonFX() {
    flywheelTalon = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEARING;
    flywheelConfig.Slot0 = ShooterConstants.FLYWHEEL_GAINS.toSlot0Configs();
    flywheelConfig.MotionMagic = ShooterConstants.FLYWHEEL_MOTION_MAGIC_CONFIGS;
    flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_INVERTED;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.StatorCurrentLimit = 130;

    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 60;
    flywheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    flywheelConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheel2Talon = new TalonFX(ShooterConstants.FLYWHEEL2_MOTOR_ID, ShooterConstants.CAN_BUS);
    flywheel2Talon.setControl(
        new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FEEDER_GEARING;
    feederConfig.Slot0 = ShooterConstants.FEEDER_GAINS.toSlot0Configs();
    feederConfig.MotorOutput.Inverted = ShooterConstants.FEEDER_INVERTED;
    feederConfig.MotionMagic = ShooterConstants.FEEDER_MOTION_MAGIC_CONFIGS;

    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.CurrentLimits.StatorCurrentLimit = 120;

    feederConfig.CurrentLimits.SupplyCurrentLimit = 50;
    feederConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    feederConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    AlertUtils.processCriticalAlert(
        flywheelConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> flywheelTalon.getConfigurator().apply(flywheelConfig)));

    AlertUtils.processCriticalAlert(
        flywheelConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> flywheel2Talon.getConfigurator().apply(flywheelConfig)));

    AlertUtils.processCriticalAlert(
        feederConfigAlert,
        !tryUntilOk(
            Constants.MAX_PHEONIX_RETRIES,
            () -> feederTalon.getConfigurator().apply(feederConfig)));

    flywheelVelocity = flywheelTalon.getVelocity();
    flywheelSupplyCurent = flywheelTalon.getSupplyCurrent();
    flywheel2SupplyCurent = flywheel2Talon.getSupplyCurrent();
    flywheelAppliedVoltage = flywheelTalon.getMotorVoltage();

    feederVelocity = feederTalon.getVelocity();
    feederSupplyCurent = feederTalon.getSupplyCurrent();
    feederAppliedVoltage = feederTalon.getMotorVoltage();

    // Initialize Status Signals
    statusSignalCollector =
        new BetterStatusSignalCollection(
            flywheelVelocity,
            flywheelSupplyCurent,
            flywheel2SupplyCurent,
            flywheelAppliedVoltage,
            feederVelocity,
            feederSupplyCurent,
            feederAppliedVoltage);
    flywheelVelocity.setUpdateFrequency(50);
    feederVelocity.setUpdateFrequency(50);
    flywheelSupplyCurent.setUpdateFrequency(20);
    flywheel2SupplyCurent.setUpdateFrequency(20);
    flywheelAppliedVoltage.setUpdateFrequency(20);
    feederSupplyCurent.setUpdateFrequency(20);
    feederAppliedVoltage.setUpdateFrequency(20);
    ParentDevice.optimizeBusUtilizationForAll(flywheelTalon, flywheel2Talon, feederTalon);
    beamBreak.start();
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
    inputs.flywheelTalonConnected = flywheelVelocity.getStatus().isOK();
    inputs.feederTalonConnected = feederVelocity.getStatus().isOK();
    inputs.beamBreakTriggered = beamBreak.getTriggeredSinceLastCheck();

    // Update Inputs
    inputs.flywheelVelocity = flywheelVelocity.getValue();
    inputs.flywheelSupplyCurrent = flywheelSupplyCurent.getValue();
    inputs.flywheel2SupplyCurrent = flywheel2SupplyCurent.getValue();
    inputs.flywheelAppliedVoltage = flywheelAppliedVoltage.getValue();

    inputs.feederVelocity = feederVelocity.getValue();
    inputs.feederSupplyCurrent = feederSupplyCurent.getValue();
    inputs.feederAppliedVoltage = feederAppliedVoltage.getValue();
  }

  public void setFlywheelSpeed(AngularVelocity speed) {
    double currentVelocity = flywheelVelocity.getValue().in(RotationsPerSecond);
    double targetVelocity = speed.in(RotationsPerSecond);
    boolean recovering =
        currentVelocity < (targetVelocity - ShooterConstants.FLYWHEEL_RECOVERY_THRESHOLD_RPS);
    flywheelTalon.setControl(
        flywheelVelocityVoltage
            .withVelocity(speed)
            .withAcceleration(
                recovering ? ShooterConstants.FLYWHEEL_RECOVERY_ACCELERATION_RPS2 : 0));
  }

  public void setFlywheelVoltage(Voltage voltage) {
    flywheelTalon.setControl(flywheelVoltageOut.withOutput(voltage));
  }

  public void setFeederVoltage(Voltage voltage) {
    feederTalon.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setDefenseMode(boolean active) {
    var flywheelLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(active ? 30.0 : 60.0)
            .withSupplyCurrentLowerLimit(active ? 30.0 : 40.0)
            .withStatorCurrentLimit(active ? 80.0 : 130.0)
            .withStatorCurrentLimitEnable(true);
    flywheelTalon.getConfigurator().apply(flywheelLimits, 0.0);
    flywheel2Talon.getConfigurator().apply(flywheelLimits, 0.0);

    var feederLimits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(active ? 25.0 : 50.0)
            .withSupplyCurrentLowerLimit(active ? 25.0 : 40.0)
            .withStatorCurrentLimit(active ? 40.0 : 80.0)
            .withStatorCurrentLimitEnable(true);
    feederTalon.getConfigurator().apply(feederLimits, 0.0);
  }

  public void setFeederSpeed(AngularVelocity speed) {
    double currentVelocity = feederVelocity.getValue().in(RotationsPerSecond);
    double targetVelocity = speed.in(RotationsPerSecond);
    boolean recovering =
        currentVelocity < (targetVelocity - ShooterConstants.FLYWHEEL_RECOVERY_THRESHOLD_RPS);
    feederTalon.setControl(
        feederVelocityVoltage
            .withVelocity(speed)
            .withAcceleration(
                recovering ? ShooterConstants.FLYWHEEL_RECOVERY_ACCELERATION_RPS2 : 0));
  }
}
