package frc.robot.subsystems.hopper;

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

public class HopperIOTalonFX implements HopperIO {
  // Alerts
  private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
  private Alert indexerConfigAlert =
      new Alert("CRITICAL", "Failed To Configure Indexer Motor", AlertType.kError);

  // Motor Controllers
  public TalonFX indexerTalon;
  public TalonFX topIndexerTalon;

  // Status Signals
  public BetterStatusSignalCollection statusSignalCollector;

  public StatusSignal<AngularVelocity> indexerVelocity;
  public StatusSignal<AngularAcceleration> indexerAcceleration;
  public StatusSignal<Current> indexerSupplyCurrent;
  public StatusSignal<Voltage> indexerVoltage;

  public StatusSignal<AngularVelocity> topIndexerVelocity;
  public StatusSignal<AngularAcceleration> topIndexerAcceleration;
  public StatusSignal<Current> topIndexerSupplyCurrent;
  public StatusSignal<Voltage> topIndexerVoltage;

  public HopperIOTalonFX() {
    this.indexerTalon = new TalonFX(HopperConstants.INDEXER_MOTOR_ID, HopperConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    indexerConfig.Feedback.SensorToMechanismRatio = HopperConstants.INDEXER_GEARING;
    indexerConfig.MotorOutput.Inverted = HopperConstants.INDEXER_MOTOR_DIRECTION;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.SupplyCurrentLimit = 30;
    indexerConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
    indexerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.35;

    if (tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES, () -> indexerTalon.getConfigurator().apply(indexerConfig))) {
      indexerConfigAlert.set(false);
    } else {
      indexerConfigAlert.set(true);
    }

    // Status Signals
    indexerVelocity = indexerTalon.getVelocity();
    indexerAcceleration = indexerTalon.getAcceleration();
    indexerSupplyCurrent = indexerTalon.getSupplyCurrent();
    indexerVoltage = indexerTalon.getMotorVoltage();

    if (HopperConstants.USE_TOP_INDEXER) {
      this.topIndexerTalon =
          new TalonFX(HopperConstants.TOP_INDEXER_MOTOR_ID, HopperConstants.CAN_BUS);

      // Configure Top Indexer TalonFX
      TalonFXConfiguration topIndexerConfig = new TalonFXConfiguration();
      topIndexerConfig.Feedback.SensorToMechanismRatio = HopperConstants.TOP_INDEXER_GEARING;
      topIndexerConfig.MotorOutput.Inverted = HopperConstants.TOP_INDEXER_MOTOR_DIRECTION;
      topIndexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      topIndexerConfig.CurrentLimits.SupplyCurrentLimit = 30;
      topIndexerConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
      topIndexerConfig.CurrentLimits.SupplyCurrentLowerTime = 0.35;

      AlertUtils.processCriticalAlert(
          indexerConfigAlert,
          !tryUntilOk(
              Constants.MAX_PHEONIX_RETRIES,
              () -> topIndexerTalon.getConfigurator().apply(topIndexerConfig)));

      topIndexerVelocity = topIndexerTalon.getVelocity();
      topIndexerAcceleration = topIndexerTalon.getAcceleration();
      topIndexerSupplyCurrent = topIndexerTalon.getSupplyCurrent();
      topIndexerVoltage = topIndexerTalon.getMotorVoltage();

      statusSignalCollector =
          new BetterStatusSignalCollection(
              indexerVelocity,
              indexerAcceleration,
              indexerSupplyCurrent,
              indexerVoltage,
              topIndexerVelocity,
              topIndexerAcceleration,
              topIndexerSupplyCurrent,
              topIndexerVoltage);
      ParentDevice.optimizeBusUtilizationForAll(indexerTalon, topIndexerTalon);
    } else {
      statusSignalCollector =
          new BetterStatusSignalCollection(
              indexerVelocity, indexerAcceleration, indexerSupplyCurrent, indexerVoltage);
      ParentDevice.optimizeBusUtilizationForAll(indexerTalon);
    }
    statusSignalCollector.setUpdateFrequencyForAll(50);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    statusSignalCollector.refreshAll();

    if (!statusSignalCollector.isAllGood()) {
      statusSignalAlert.set(true);
      statusSignalAlert.setText(
          "Indexer TalonFX Status Signal Error: " + statusSignalCollector.getBadSignalsString());
    } else {
      statusSignalAlert.set(false);
    }

    inputs.indexerTalonConnected = indexerTalon.isConnected();

    inputs.indexerVelocity = indexerVelocity.getValue();
    inputs.indexerAcceleration = indexerAcceleration.getValue();
    inputs.indexerSupplyCurrent = indexerSupplyCurrent.getValue();
    inputs.indexerAppliedVoltage = indexerVoltage.getValue();

    if (HopperConstants.USE_TOP_INDEXER) {
      inputs.topIndexerTalonConnected = topIndexerTalon.isConnected();
      inputs.topIndexerVelocity = topIndexerVelocity.getValue();
      inputs.topIndexerAcceleration = topIndexerAcceleration.getValue();
      inputs.topIndexerSupplyCurrent = topIndexerSupplyCurrent.getValue();
      inputs.topIndexerAppliedVoltage = topIndexerVoltage.getValue();
    }
  }

  public void setIndexerVoltage(Voltage voltage) {
    indexerTalon.setVoltage(voltage.in(Volts));
    if (HopperConstants.USE_TOP_INDEXER) {
      topIndexerTalon.setVoltage(voltage.in(Volts));
    }
  }

  public void setTopIndexerVoltage(Voltage voltage) {
    if (HopperConstants.USE_TOP_INDEXER) {
      topIndexerTalon.setVoltage(voltage.in(Volts));
    }
  }
}
