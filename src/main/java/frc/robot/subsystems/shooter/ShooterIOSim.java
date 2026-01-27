package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  // Physics constants
  private static final double FLYWHEEL_MASS_KG = 1.134; // 2.5 lbs
  private static final double FLYWHEEL_RADIUS_M = Units.inchesToMeters(2.0); // 4" diameter
  private static final double FEEDER_MASS_KG = 0.227; // 0.5 lbs
  private static final double FEEDER_RADIUS_M = Units.inchesToMeters(2.0); // 4" diameter

  // Moment of inertia for a disk: I = 0.5 * m * r^2
  private static final double FLYWHEEL_MOI =
      0.5 * FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M;
  private static final double FEEDER_MOI = 0.5 * FEEDER_MASS_KG * FEEDER_RADIUS_M * FEEDER_RADIUS_M;

  // Create TalonFX objects for simulation
  private final TalonFX flywheelTalon;
  private final TalonFX feederTalon;

  // Simulated motor controllers
  private final FlywheelSim flywheelSim;
  private final FlywheelSim feederSim;

  // Phoenix 6 simulation states
  private final TalonFXSimState flywheelSimState;
  private final TalonFXSimState feederSimState;

  public ShooterIOSim() {
    // Create TalonFX objects (these exist only in simulation)
    flywheelTalon = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CAN_BUS);

    // Configure TalonFX Motors
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEARING;
    flywheelConfig.Slot0 = ShooterConstants.SIM_FLYWHEEL_GAINS.toSlot0Configs();
    flywheelConfig.MotionMagic = ShooterConstants.FLYWHEEL_MOTION_MAGIC_CONFIGS;
    flywheelConfig.MotorOutput.Inverted = ShooterConstants.FLYWHEEL_INVERTED;

    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FEEDER_GEARING;
    feederConfig.MotorOutput.Inverted = ShooterConstants.FEEDER_INVERTED;

    tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES, () -> flywheelTalon.getConfigurator().apply(flywheelConfig));

    tryUntilOk(
        Constants.MAX_PHEONIX_RETRIES, () -> feederTalon.getConfigurator().apply(feederConfig));

    // Get Phoenix 6 simulation states
    flywheelSimState = flywheelTalon.getSimState();
    feederSimState = feederTalon.getSimState();

    // Set supply voltage for sim
    flywheelSimState.setSupplyVoltage(12.0);
    feederSimState.setSupplyVoltage(12.0);

    // Create flywheel simulation
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), FLYWHEEL_MOI, ShooterConstants.FLYWHEEL_GEARING),
            DCMotor.getKrakenX60(1));

    // Create feeder simulation
    feederSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1), FEEDER_MOI, ShooterConstants.FEEDER_GEARING),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Get motor voltages from Phoenix 6 sim states
    double flywheelVoltage = flywheelSimState.getMotorVoltage();
    double feederVoltage = feederSimState.getMotorVoltage();

    // Update physics simulations
    flywheelSim.setInputVoltage(flywheelVoltage);
    feederSim.setInputVoltage(feederVoltage);

    flywheelSim.update(0.02); // 20ms period
    feederSim.update(0.02);

    // FlywheelSim gives us MECHANISM velocity (after gearing)
    // Convert to ROTOR velocity (before gearing) by multiplying by gear ratio
    // Convert rad/s to rotations/s
    double flywheelMechanismRPS = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double feederMechanismRPS = feederSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Convert mechanism speed to rotor speed: rotorSpeed = mechanismSpeed * gearRatio
    double flywheelRotorRPS = flywheelMechanismRPS * ShooterConstants.FLYWHEEL_GEARING;
    double feederRotorRPS = feederMechanismRPS * ShooterConstants.FEEDER_GEARING;

    flywheelSimState.setRotorVelocity(flywheelRotorRPS);
    feederSimState.setRotorVelocity(feederRotorRPS);

    // Update rotor positions (integrate velocity)
    flywheelSimState.addRotorPosition(flywheelRotorRPS * 0.02);
    feederSimState.addRotorPosition(feederRotorRPS * 0.02);

    // Rest of the method stays the same...
    inputs.flywheelTalonConnected = true;
    inputs.feederTalonConnected = true;

    inputs.flywheelVelocity = flywheelTalon.getVelocity().getValue();
    inputs.flywheelAcceleration = flywheelTalon.getAcceleration().getValue();
    inputs.flywheelAppliedVoltage = flywheelTalon.getMotorVoltage().getValue();
    inputs.flywheelAppliedCurrent = flywheelTalon.getSupplyCurrent().getValue();

    inputs.feederVelocity = feederTalon.getVelocity().getValue();
    inputs.feederAcceleration = feederTalon.getAcceleration().getValue();
    inputs.feederAppliedVoltage = feederTalon.getMotorVoltage().getValue();
    inputs.feederAppliedCurrent = feederTalon.getSupplyCurrent().getValue();
  }

  @Override
  public void setFlywheelSpeed(AngularVelocity speed) {
    // Phoenix 6 handles closed-loop control automatically in simulation
    flywheelTalon.setControl(
        new com.ctre.phoenix6.controls.VelocityVoltage(0).withVelocity(speed).withSlot(0));
  }

  @Override
  public void setFlywheelVoltage(Voltage voltage) {
    flywheelTalon.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setFeederVoltage(Voltage voltage) {
    feederTalon.setVoltage(voltage.in(Volts));
  }
}
