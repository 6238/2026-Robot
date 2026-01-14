package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private TalonFX shooterTalon;
  private TalonFX feederTalon;

  // Status Signal Management
  public StatusSignalCollection statusSignalCollector;

  // Shooter Inputs
  public StatusSignal<AngularVelocity> shooterVelocity;
  public StatusSignal<AngularAcceleration> shooterAcceleration;
  public StatusSignal<Current> shooterAppliedCurent;
  public StatusSignal<Voltage> shooterAppliedVoltage;

  // Feeder Inputs
  public StatusSignal<AngularVelocity> feederVelocity;
  public StatusSignal<AngularAcceleration> feederAcceleration;
  public StatusSignal<Current> feederAppliedCurent;
  public StatusSignal<Voltage> feederAppliedVoltage;

  // Control Requests
  public PositionVoltage hoodPositionVoltage = new PositionVoltage(0).withSlot(0);
  public VelocityVoltage shooterVelocityVoltage = new VelocityVoltage(0).withSlot(0);

  public ShooterIOTalonFX() {
    shooterTalon = new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);

    tryUntilOk(5, () -> hoodTalon.setPosition(Degrees.of(22)));

    // BaseStatusSignal.setUpdateFrequencyForAll(
    //     50.0,
    //     shooterVelocity,
    //     shooterAcceleration,
    //     shooterAppliedCurent,
    //     shooterAppliedVoltage,
    //     feederVelocity,
    //     feederAcceleration,
    //     feederAppliedCurent,
    //     feederAppliedVoltage,
    //     hoodAngle,
    //     hoodVelocity,
    //     hoodAcceleration,
    //     hoodAppliedCurent,
    //     hoodAppliedVoltage);
    ParentDevice.optimizeBusUtilizationForAll(shooterTalon, feederTalon, hoodTalon);
  }

  public void updateInputs(ShooterIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        shooterVelocity,
        shooterAcceleration,
        shooterAppliedCurent,
        shooterAppliedVoltage,
        feederVelocity,
        feederAcceleration,
        feederAppliedCurent,
        feederAppliedVoltage,
        hoodAngle,
        hoodVelocity,
        hoodAcceleration,
        hoodAppliedCurent,
        hoodAppliedVoltage);

    // Are motors connected?
    inputs.shooterTalonConnected = shooterTalon.isConnected();
    inputs.feederTalonConnected = feederTalon.isConnected();
    inputs.hoodTalonConnected = shooterTalon.isConnected();

    // Update Inputs
    inputs.shooterVelocity = shooterTalon.getVelocity().getValue();
    inputs.shooterAcceleration = shooterTalon.getAcceleration().getValue();
    inputs.shooterAppliedCurrent = shooterTalon.getSupplyCurrent().getValue();
    inputs.shooterAppliedVoltage = shooterTalon.getSupplyVoltage().getValue();

    inputs.feederVelocity = feederTalon.getVelocity().getValue();
    inputs.fAcceleration = feederTalon.getAcceleration().getValue();
    inputs.feederAppliedCurrent = feederTalon.getSupplyCurrent().getValue();
    inputs.feederAppliedVoltage = feederTalon.getSupplyVoltage().getValue();

    inputs.hoodAngle = hoodTalon.getPosition().getValue();
    inputs.hoodVelocity = hoodTalon.getVelocity().getValue();
    inputs.hoodAcceleration = hoodTalon.getAcceleration().getValue();
    inputs.hoodAppliedCurrent = hoodTalon.getSupplyCurrent().getValue();
    inputs.hoodAppliedVoltage = hoodTalon.getSupplyVoltage().getValue();
  }

  public void setShooterRPM(AngularVelocity rpm) {
    shooterTalon.setControl(shooterVelocityVoltage.withVelocity(rpm));
  }

  public void setShooterVoltage(Voltage voltage) {
    shooterTalon.setVoltage(voltage.in(Volts));
  }

  public void setFeederVoltage(Voltage voltage) {
    feederTalon.setVoltage(voltage.in(Volts));
  }

  public void setHoodAngle(Angle angle) {
    hoodTalon.setControl(hoodPositionVoltage.withPosition(angle));
  }
}
