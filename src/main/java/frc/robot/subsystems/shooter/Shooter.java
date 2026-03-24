package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AlertUtils;
import frc.robot.util.BatteryLogger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public ShooterIO io;
  public ShooterIOInputsAutoLogged inputs;

  public AngularVelocity targetFlywheelVelocity;
  public AngularVelocity targetFeederVelocity;

  private BatteryLogger batteryLogger;

  public void setBatteryLogger(BatteryLogger batteryLogger) {
    this.batteryLogger = batteryLogger;
  }

  public Alert shooterMotorConnectedAlert =
      new Alert("Critical", "Shooter Flywheel Motor Disconnected", AlertType.kError);
  public Alert feederMotorConnectedAlert =
      new Alert("Critical", "Shooter Feeder Motor Disconnected", AlertType.kError);

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();

    this.targetFlywheelVelocity = RotationsPerSecond.of(0);
    this.targetFeederVelocity = RotationsPerSecond.of(0);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Feeder velocity compensation: boost flywheel speed proportional to feeder deficit
    // AngularVelocity compensatedFlywheelVelocity = targetFlywheelVelocity;
    // if (targetFlywheelVelocity.in(RotationsPerSecond) > 0
    //     && targetFeederVelocity.in(RotationsPerSecond) > 0) {
    //   double targetRPS = targetFeederVelocity.in(RotationsPerSecond);
    //   double feederError = targetRPS - inputs.feederVelocity.in(RotationsPerSecond);
    //   double compensation = 0;
    //   if (feederError > 0.15 * targetRPS) {
    //     compensation = feederError * ShooterConstants.FEEDER_COMPENSATION_GAIN.get();
    //   }
    //   compensatedFlywheelVelocity =
    //       RotationsPerSecond.of(targetFlywheelVelocity.in(RotationsPerSecond) + compensation);
    //   io.setFlywheelSpeed(compensatedFlywheelVelocity);
    // }

    Logger.recordOutput("Shooter/targetVelocity", targetFlywheelVelocity.in(RotationsPerSecond));
    // Logger.recordOutput(
    //     "Shooter/compensatedVelocity", compensatedFlywheelVelocity.in(RotationsPerSecond));
    Logger.recordOutput("Shooter/currentVelocity", inputs.flywheelVelocity.in(RotationsPerSecond));

    if (batteryLogger != null) {
      batteryLogger.reportCurrentUsage(
          "Shooter/Flywheel",
          inputs.flywheelSupplyCurrent.in(Amps),
          inputs.flywheel2SupplyCurrent.in(Amps));
      batteryLogger.reportCurrentUsage("Shooter/Feeder", inputs.feederSupplyCurrent.in(Amps));
    }

    // update alerts based on motor connection status
    AlertUtils.processCriticalAlert(shooterMotorConnectedAlert, !inputs.flywheelTalonConnected);
    AlertUtils.processCriticalAlert(feederMotorConnectedAlert, !inputs.feederTalonConnected);
  }

  public Command setFlywheelRPM(Supplier<AngularVelocity> speed) {
    return runOnce(
        () -> {
          setFlywheelRPM(speed.get());
        });
  }

  public void setFlywheelRPM(AngularVelocity speed) {
    io.setFlywheelSpeed(speed);
    this.targetFlywheelVelocity = speed;
  }

  public Command setFlywheelVoltage(Supplier<Voltage> voltage) {
    return runOnce(
        () -> {
          setFlywheelVoltage(voltage.get());
        });
  }

  public void setFlywheelVoltage(Voltage voltage) {
    io.setFlywheelVoltage(voltage);
    this.targetFlywheelVelocity = RotationsPerSecond.of(0);
    this.targetFeederVelocity = RotationsPerSecond.of(0);
  }

  public Command setFeederVoltage(Supplier<Voltage> voltage) {
    return runOnce(() -> setFeederVoltage(voltage.get()));
  }

  public void setFeederVoltage(Voltage voltage) {
    io.setFeederVoltage(voltage);
    this.targetFeederVelocity = RotationsPerSecond.of(0);
  }

  public Command setFeederSpeed(Supplier<AngularVelocity> speed) {
    return runOnce(() -> setFeederSpeed(speed.get()));
  }

  public void setFeederSpeed(AngularVelocity speed) {
    io.setFeederSpeed(speed);
    this.targetFeederVelocity = speed;
  }

  public boolean flywheelUpToSpeed() {
    return inputs.flywheelVelocity.isNear(
        this.targetFlywheelVelocity, ShooterConstants.FLYWHEEL_TOLERANCE_BEFORE_SHOT);
  }

  public boolean flywheelUpToSpeed(AngularVelocity tolerance) {
    return inputs.flywheelVelocity.isNear(this.targetFlywheelVelocity, tolerance);
  }

  public boolean feederUpToSpeed() {
    return inputs.feederVelocity.isNear(
        this.targetFeederVelocity, ShooterConstants.FEEDER_TOLERANCE_BEFORE_SHOT);
  }

  public AngularVelocity getCurrentFlywheelSpeed() {
    return inputs.flywheelVelocity;
  }

  /**
   * Runs a two-phase flywheel system identification routine and prints kS, kV, and kA to the
   * console and DriverStation when complete. Run this as an autonomous routine.
   *
   * <p>Phase 1 (quasistatic): slowly ramps voltage to collect steady-state voltage vs. velocity
   * data, then fits kS and kV via linear regression (V = kS + kV * ω).
   *
   * <p>Phase 2 (dynamic step): applies a fixed voltage step to measure acceleration, then computes
   * kA = avg((V - kS - kV * ω) / α).
   */
  public Command flywheelSysId() {
    final double QUASISTATIC_RAMP_RATE = 0.75; // V/s
    final double QUASISTATIC_DURATION = 12.0; // seconds (~9 V at end)
    final double QUASISTATIC_MIN_VELOCITY = 1.0; // RPS, skip dead-zone data
    final double SPINDOWN_WAIT = 15.0; // seconds between phases
    final double DYNAMIC_VOLTAGE = 6.0; // V
    final double DYNAMIC_DURATION = 3.0; // seconds

    List<Double> qsVoltages = new ArrayList<>();
    List<Double> qsVelocities = new ArrayList<>();
    List<Double> dynVoltages = new ArrayList<>();
    List<Double> dynVelocities = new ArrayList<>();
    List<Double> dynAccelerations = new ArrayList<>();
    double[] startTime = {0};
    double[] prevTime = {0};
    double[] prevVelocity = {0};
    double[] kSResult = {0};
    double[] kVResult = {0};

    return Commands.sequence(
            // ── Phase 1: Quasistatic ramp ────────────────────────────────────
            runOnce(
                () -> {
                  qsVoltages.clear();
                  qsVelocities.clear();
                  targetFlywheelVelocity = RotationsPerSecond.of(0);
                  targetFeederVelocity = RotationsPerSecond.of(0);
                  startTime[0] = Timer.getFPGATimestamp();
                  System.out.println("[FlywheelSysId] Phase 1: quasistatic ramp starting...");
                }),
            run(() -> {
                  double elapsed = Timer.getFPGATimestamp() - startTime[0];
                  double voltage = QUASISTATIC_RAMP_RATE * elapsed;
                  io.setFlywheelVoltage(Volts.of(voltage));
                  double vel = inputs.flywheelVelocity.in(RotationsPerSecond);
                  if (vel > QUASISTATIC_MIN_VELOCITY) {
                    qsVoltages.add(inputs.flywheelAppliedVoltage.in(Volts));
                    qsVelocities.add(vel);
                  }
                })
                .withTimeout(QUASISTATIC_DURATION),
            runOnce(
                () -> {
                  io.setFlywheelVoltage(Volts.of(0));
                  System.out.println(
                      "[FlywheelSysId] Phase 1 done — "
                          + qsVoltages.size()
                          + " data points. Spinning down...");
                }),
            Commands.waitSeconds(SPINDOWN_WAIT),

            // ── Phase 2: Dynamic step ─────────────────────────────────────────
            runOnce(
                () -> {
                  dynVoltages.clear();
                  dynVelocities.clear();
                  dynAccelerations.clear();
                  startTime[0] = Timer.getFPGATimestamp();
                  prevTime[0] = startTime[0];
                  prevVelocity[0] = inputs.flywheelVelocity.in(RotationsPerSecond);
                  System.out.println("[FlywheelSysId] Phase 2: dynamic step starting...");
                }),
            run(() -> {
                  io.setFlywheelVoltage(Volts.of(DYNAMIC_VOLTAGE));
                  double now = Timer.getFPGATimestamp();
                  double dt = now - prevTime[0];
                  double vel = inputs.flywheelVelocity.in(RotationsPerSecond);
                  if (dt > 0.005) {
                    double accel = (vel - prevVelocity[0]) / dt;
                    dynVoltages.add(inputs.flywheelAppliedVoltage.in(Volts));
                    dynVelocities.add(vel);
                    dynAccelerations.add(accel);
                    prevTime[0] = now;
                    prevVelocity[0] = vel;
                  }
                })
                .withTimeout(DYNAMIC_DURATION),

            // ── Compute and print results ──────────────────────────────────────
            runOnce(
                () -> {
                  io.setFlywheelVoltage(Volts.of(0));
                  targetFlywheelVelocity = RotationsPerSecond.of(0);

                  // Linear regression: V = kS + kV * ω
                  int n = qsVoltages.size();
                  if (n < 10) {
                    String msg =
                        "[FlywheelSysId] ERROR: only "
                            + n
                            + " quasistatic points — increase ramp duration or lower"
                            + " MIN_VELOCITY threshold.";
                    System.out.println(msg);
                    DriverStation.reportWarning(msg, false);
                    return;
                  }
                  double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                  for (int i = 0; i < n; i++) {
                    double x = qsVelocities.get(i);
                    double y = qsVoltages.get(i);
                    sumX += x;
                    sumY += y;
                    sumXY += x * y;
                    sumX2 += x * x;
                  }
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                  double kS = (sumY - kV * sumX) / n;
                  kSResult[0] = kS;
                  kVResult[0] = kV;

                  // kA = avg((V - kS - kV * ω) / α) for points with meaningful acceleration
                  int m = dynVoltages.size();
                  double kASum = 0;
                  int kACount = 0;
                  for (int i = 0; i < m; i++) {
                    double accel = dynAccelerations.get(i);
                    if (Math.abs(accel) > 1.0) {
                      kASum += (dynVoltages.get(i) - kS - kV * dynVelocities.get(i)) / accel;
                      kACount++;
                    }
                  }
                  double kA = kACount > 0 ? kASum / kACount : Double.NaN;

                  String results =
                      String.format(
                          "%n=== Flywheel SysId Results ===%n"
                              + "  kS = %.5f  V%n"
                              + "  kV = %.5f  V / (rot/s)%n"
                              + "  kA = %.5f  V / (rot/s²)%n"
                              + "  Quasistatic pts : %d%n"
                              + "  Dynamic pts     : %d  (used for kA: %d)%n"
                              + "==============================",
                          kS, kV, kA, n, m, kACount);
                  System.out.println(results);
                  DriverStation.reportWarning(results, false);
                }))
        .withName("Flywheel SysId");
  }
}
