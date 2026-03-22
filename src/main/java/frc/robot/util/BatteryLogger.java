package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Tracks per-motor current draw and integrates energy usage over the match. */
public class BatteryLogger {
  private double totalCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;

  private final Map<String, Double> subsystemCurrents = new HashMap<>();
  private final Map<String, Double> subsystemPowers = new HashMap<>();
  private final Map<String, Double> subsystemEnergies = new HashMap<>();

  /**
   * Reports current usage for a named motor or group. Aggregates up the key hierarchy using both
   * '/' and '-' as delimiters. E.g. "Drive/Module0-Drive" rolls up into "Drive/Module0" and
   * "Drive".
   *
   * @param key Hierarchical name (e.g. "Shooter/Flywheel", "Drive/Module0-Drive")
   * @param amps One or more current readings in amps (absolute values are summed)
   */
  public void reportCurrentUsage(String key, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.abs(amp);

    double batteryVoltage = RobotController.getBatteryVoltage();
    double power = totalAmps * batteryVoltage;
    double energy = power * Constants.loopPeriodSecs;

    totalCurrent += totalAmps;
    totalPower += power;
    totalEnergy += energy;

    subsystemCurrents.put(key, totalAmps);
    subsystemPowers.put(key, power);
    subsystemEnergies.merge(key, energy, Double::sum);

    // Aggregate into parent keys (each segment separated by '/' or '-')
    String[] parts = key.split("/|-");
    StringBuilder subkey = new StringBuilder();
    for (int i = 0; i < parts.length - 1; i++) {
      if (i > 0) subkey.append("/");
      subkey.append(parts[i]);
      String parentKey = subkey.toString();
      subsystemCurrents.merge(parentKey, totalAmps, Double::sum);
      subsystemPowers.merge(parentKey, power, Double::sum);
      subsystemEnergies.merge(parentKey, energy, Double::sum);
    }
  }

  /**
   * Logs all accumulated data and resets per-loop current/power totals. Call this once per loop
   * after CommandScheduler.run().
   */
  public void periodicAfterScheduler() {
    // Fixed overhead estimates for non-motor devices
    reportCurrentUsage("Controls/roboRIO", RobotController.getInputCurrent());
    reportCurrentUsage("Controls/CANcoders", 0.05 * 4);
    reportCurrentUsage("Controls/Pigeon", 0.04);
    reportCurrentUsage("Controls/CANivore", 0.03);
    reportCurrentUsage("Controls/Radio", 0.5);

    Logger.recordOutput("EnergyLogger/TotalCurrentAmps", totalCurrent);
    Logger.recordOutput("EnergyLogger/TotalPowerWatts", totalPower);
    Logger.recordOutput("EnergyLogger/TotalEnergyWattHours", joulesToWattHours(totalEnergy));
    Logger.recordOutput("EnergyLogger/BatteryVoltage", RobotController.getBatteryVoltage());

    for (var entry : subsystemCurrents.entrySet()) {
      Logger.recordOutput("EnergyLogger/Current/" + entry.getKey(), entry.getValue());
      subsystemCurrents.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemPowers.entrySet()) {
      Logger.recordOutput("EnergyLogger/Power/" + entry.getKey(), entry.getValue());
      subsystemPowers.put(entry.getKey(), 0.0);
    }
    for (var entry : subsystemEnergies.entrySet()) {
      Logger.recordOutput(
          "EnergyLogger/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()));
    }

    // Reset per-loop totals (energy accumulates; current/power reset each loop)
    totalPower = 0.0;
    totalCurrent = 0.0;
  }

  public double getTotalCurrent() {
    return totalCurrent;
  }

  public double getTotalPower() {
    return totalPower;
  }

  /** Returns total integrated energy in joules (not watt-hours). */
  public double getTotalEnergy() {
    return totalEnergy;
  }

  private double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }
}
