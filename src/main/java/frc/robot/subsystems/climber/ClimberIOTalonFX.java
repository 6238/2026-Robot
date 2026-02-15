package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.linearStdDevBaseline;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.AlertUtils;
import frc.robot.util.BetterStatusSignalCollection;

public class ClimberIOTalonFX implements ClimberIO {
    // Alerts
    private Alert statusSignalAlert = new Alert("ISSUE", "Status Signal Error", AlertType.kError);
    private Alert climberConfigAlert = new Alert("CRITICAL", "Failed To Configure Climber Motor", AlertType.kError);

    // Motor Talons
    private TalonFX climberTalon;

    // Status Signal Management
    public BetterStatusSignalCollection statusSignalCollector;

    // Climber Status Signals
    public StatusSignal<Angle> climberPosition;
    public StatusSignal<AngularVelocity> climberVelocity;
    public StatusSignal<AngularAcceleration> climberAcceleration;
    public StatusSignal<Current> climberAppliedCurent;
    public StatusSignal<Voltage> climberAppliedVoltage;

    public MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0).withSlot(0);

    public ClimberIOTalonFX() {
        climberTalon = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID, ClimberConstants.CAN_BUS);

        // Configure TalonFX Motor
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        climberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.CLIMBER_GEARING;
        climberConfig.Slot0 = ClimberConstants.CLIMBER_GAINS.toSlot0Configs();
        climberConfig.MotionMagic = ClimberConstants.MOTION_MAGIC_CONFIGS;
        climberConfig.MotorOutput.Inverted = ClimberConstants.CLIMBER_MOTOR_DIRECTION;

        AlertUtils.processCriticalAlert(
                climberConfigAlert,
                !tryUntilOk(
                        Constants.MAX_PHEONIX_RETRIES,
                        () -> climberTalon.getConfigurator().apply(climberConfig)));

        climberPosition = climberTalon.getPosition();
        climberVelocity = climberTalon.getVelocity();
        climberAcceleration = climberTalon.getAcceleration();
        climberAppliedCurent = climberTalon.getStatorCurrent();
        climberAppliedVoltage = climberTalon.getMotorVoltage();

        // Initialize Status Signals
        statusSignalCollector = new BetterStatusSignalCollection(
                climberVelocity,
                climberAcceleration,
                climberAppliedCurent,
                climberAppliedVoltage);
        statusSignalCollector.setUpdateFrequencyForAll(50);
        ParentDevice.optimizeBusUtilizationForAll(climberTalon);
    }

    public void updateInputs(ClimberIOInputs inputs) {
        statusSignalCollector.refreshAll();

        if (!statusSignalCollector.isAllGood()) {
            statusSignalAlert.set(true);
            statusSignalAlert.setText(
                    "Climber TalonFX Status Signal Error: " + statusSignalCollector.getBadSignalsString());
        } else {
            statusSignalAlert.set(false);
        }

        inputs.climberTalonConnected = climberTalon.isConnected();

        inputs.climberPosition = ClimberConstants.DISTANCE_GEARING_UNIT.of(climberPosition.getValue().in(Rotations));
        inputs.climberVelocity = ClimberConstants.LINEAR_VELOCITY_GEARING_UNIT
                .of(climberVelocity.getValue().in(RotationsPerSecond));
        inputs.climberAcceleration = ClimberConstants.LINEAR_ACCELERATION_GEARING_UNIT
                .of(climberAcceleration.getValue().in(RotationsPerSecondPerSecond));

        inputs.climberCurrent = climberAppliedCurent.getValue();
        inputs.climberAppliedVoltage = climberAppliedVoltage.getValue();
    }

    public void setPosition(Distance position) {
        climberTalon.setControl(motionMagicVoltageRequest
                .withPosition(Rotations.of(position.in(ClimberConstants.DISTANCE_GEARING_UNIT))));
    }
}
