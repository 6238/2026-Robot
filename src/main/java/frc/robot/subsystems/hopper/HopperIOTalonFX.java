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
import frc.robot.util.BetterStatusSignalCollection;

public class HopperIOTalonFX implements HopperIO {
    // Alerts
    private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
    private Alert indexerConfigAlert = new Alert("CRITICAL", "Failed To Configure Indexer Motor", AlertType.kError);

    // Motor Controllers
    public TalonFX indexerTalon;

    // Status Signals
    public BetterStatusSignalCollection statusSignalCollector;

    public StatusSignal<AngularVelocity> indexerVelocity;
    public StatusSignal<AngularAcceleration> indexerAcceleration;
    public StatusSignal<Current> indexerCurrent;
    public StatusSignal<Voltage> indexerVoltage;

    public HopperIOTalonFX() {
        this.indexerTalon = new TalonFX(HopperConstants.INDEXER_MOTOR_ID, HopperConstants.CAN_BUS);

        // Configure TalonFX Motors
        TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
        indexerConfig.Feedback.SensorToMechanismRatio = HopperConstants.INDEXER_GEARING;
        indexerConfig.MotorOutput.Inverted = HopperConstants.INDEXER_MOTOR_DIRECTION;

        if (tryUntilOk(Constants.MAX_PHEONIX_RETRIES, () -> indexerTalon.getConfigurator().apply(indexerConfig))) {
            indexerConfigAlert.set(false);
        } else {
            indexerConfigAlert.set(true);
        }

        // Status Signals
        indexerVelocity = indexerTalon.getVelocity();
        indexerAcceleration = indexerTalon.getAcceleration();
        indexerCurrent = indexerTalon.getStatorCurrent();
        indexerVoltage = indexerTalon.getMotorVoltage();

        statusSignalCollector = new BetterStatusSignalCollection(indexerVelocity, indexerAcceleration, indexerCurrent, indexerVoltage);
        ParentDevice.optimizeBusUtilizationForAll(indexerTalon);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        statusSignalCollector.refreshAll();

        if (!statusSignalCollector.isAllGood()) {
            statusSignalAlert.set(true);
            statusSignalAlert.setText(
                "Indexer TalonFX Status Signal Error: "
                    + statusSignalCollector.getBadSignalsString());
        } else {
            statusSignalAlert.set(false);
        }

        inputs.indexerTalonConnected = indexerTalon.isConnected();

        inputs.hopperVelocity = indexerVelocity.getValue();
        inputs.hopperAcceleration = indexerAcceleration.getValue();
        inputs.hopperAppliedCurrent = indexerCurrent.getValue();
        inputs.hopperAppliedVoltage = indexerVoltage.getValue();
    }

    public void setIndexerVoltage(Voltage voltage) {
        indexerTalon.setVoltage(voltage.in(Volts));
    }
}
