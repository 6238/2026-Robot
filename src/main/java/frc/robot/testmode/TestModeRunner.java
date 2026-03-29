package frc.robot.testmode;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.BuildConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Shooter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import org.littletonrobotics.junction.Logger;

/**
 * Builds and runs the robot test-mode sequence. Actuates every motor surface at a fixed test
 * voltage, measures the resulting velocity, compares against a stored baseline, and writes an HTML
 * report to the USB drive.
 *
 * <p>This class must only be instantiated from {@code testInit()}. No static mutable state is used,
 * so there is zero overhead during teleop/auto.
 */
public class TestModeRunner {

  // ── Test voltages / outputs ──────────────────────────────────────────────
  static final double DRIVE_TEST_VOLTAGE = 2.0; // V, open-loop via runCharacterization
  static final double STEER_TEST_OUTPUT = 2.0; // duty-cycle (-1 to 1)
  static final double FLYWHEEL_TEST_VOLTAGE = 3.0; // V
  static final double FEEDER_TEST_VOLTAGE = 3.0; // V
  static final double INDEXER_TEST_VOLTAGE = 5.0; // V
  static final double ROLLER_TEST_VOLTAGE = 3.0; // V
  static final double PIVOT_TEST_VOLTAGE = 2.0; // V (same as PIVOT_PRELOAD_VOLTAGE_VOLTS)

  // ── Timing ──────────────────────────────────────────────────────────────
  static final double STEP_DURATION_S = 3.0;
  static final double SHORT_STEP_DURATION_S = 2.0;
  static final double WARMUP_S = 0.5; // ignore first 0.5 s while motor spins up
  static final double STOP_GAP_S = 0.5;
  static final double PIVOT_SETUP_WAIT_S = 2.5; // wait for pivot to reach INTAKE_DOWN
  static final double PIVOT_SWEEP_TIMEOUT_S = 6.0; // safety timeout for full-range sweep

  // ── Pass/fail thresholds (ratio of measured/baseline) ───────────────────
  static final double WARN_BELOW = 0.75; // WARN if measured < 75% of baseline
  static final double FAIL_BELOW = 0.50; // FAIL if measured < 50% of baseline
  static final double WARN_ABOVE = 1.30; // WARN if measured > 130% of baseline

  // ── Persistence ─────────────────────────────────────────────────────────
  static final String BASELINE_PATH = "/home/lvuser/test-baseline.properties";

  // ── Instance state (fresh per test run) ─────────────────────────────────
  private final Map<String, List<Double>> allSamples = new LinkedHashMap<>();
  private final List<MotorResult> results = new ArrayList<>();
  private Properties baseline = new Properties();
  private boolean saveCurrentAsBaseline = false;

  // ── Public API ───────────────────────────────────────────────────────────

  /**
   * Returns a {@link Command} that runs the full motor test sequence. Call this once per {@code
   * testInit()}.
   */
  public Command buildTestSequence(
      Drive drive,
      Shooter shooter,
      Hopper hopper,
      IntakeRoller intakeRoller,
      IntakePivot intakePivot) {

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  loadBaseline();
                  SmartDashboard.putBoolean("TestMode/SaveCurrentAsBaseline", false);
                  saveCurrentAsBaseline =
                      SmartDashboard.getBoolean("TestMode/SaveCurrentAsBaseline", false);
                  Logger.recordOutput("TestMode/ActiveStep", "INIT");
                  System.out.println("[TestMode] === Starting Robot Test Sequence ===");
                  DriverStation.reportWarning("[TestMode] Starting test sequence", false);
                }),

            // Motor steps
            driveWheelStep(drive),
            driveSteerStep(drive),
            flywheelStep(shooter),
            feederStep(shooter),
            hopperStep(hopper),
            rollerStep(intakeRoller),
            pivotStep(intakePivot),

            // Finalize
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "COMPLETE");
                  if (saveCurrentAsBaseline || !hasBaseline()) {
                    saveBaseline();
                  }
                  String html =
                      TestModeReport.generate(
                          results, RobotController.getSerialNumber(), BuildConstants.GIT_SHA);
                  TestModeReport.writeToDisk(html);
                  TestModeHttpServer.serveReport(html);
                  System.out.println("[TestMode] === Test Sequence Complete ===");
                  DriverStation.reportWarning(
                      "[TestMode] Complete — check HTML report on USB drive", false);
                }))
        .withName("Robot Test Sequence");
  }

  // ── Step builders ────────────────────────────────────────────────────────

  private Command driveWheelStep(Drive drive) {
    final String step = "DRIVE_WHEELS";
    final String[] keys = {"Module0_Drive", "Module1_Drive", "Module2_Drive", "Module3_Drive"};
    final double[] elapsed = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> announceStep(step)),
        Commands.run(
                () -> {
                  drive.runCharacterization(DRIVE_TEST_VOLTAGE);
                  elapsed[0] += 0.02;
                  if (elapsed[0] > WARMUP_S) {
                    for (int i = 0; i < 4; i++) {
                      double vel = Math.abs(drive.getModuleDriveVelocityRadPerSec(i));
                      recordSample(step, keys[i], vel);
                      Logger.recordOutput("TestMode/LiveVelocity/" + keys[i], vel);
                    }
                  }
                },
                drive)
            .withTimeout(STEP_DURATION_S),
        Commands.runOnce(() -> drive.stop(), drive),
        Commands.runOnce(() -> finalizeStep(step, keys)),
        Commands.waitSeconds(STOP_GAP_S));
  }

  private Command driveSteerStep(Drive drive) {
    final String step = "DRIVE_STEER";
    final String[] keys = {"Module0_Steer", "Module1_Steer", "Module2_Steer", "Module3_Steer"};
    final double[] elapsed = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> announceStep(step)),
        Commands.run(
                () -> {
                  drive.runSteerOpenLoop(STEER_TEST_OUTPUT);
                  elapsed[0] += 0.02;
                  if (elapsed[0] > WARMUP_S) {
                    for (int i = 0; i < 4; i++) {
                      double vel = Math.abs(drive.getModuleSteerVelocityRadPerSec(i));
                      recordSample(step, keys[i], vel);
                      Logger.recordOutput("TestMode/LiveVelocity/" + keys[i], vel);
                    }
                  }
                },
                drive)
            .withTimeout(STEP_DURATION_S),
        Commands.runOnce(() -> drive.runSteerOpenLoop(0), drive),
        Commands.runOnce(() -> finalizeStep(step, keys)),
        Commands.waitSeconds(STOP_GAP_S));
  }

  private Command flywheelStep(Shooter shooter) {
    final String step = "SHOOTER_FLYWHEEL";
    final String[] keys = {"Flywheel"};
    final double[] elapsed = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> announceStep(step)),
        Commands.run(
                () -> {
                  shooter.io.setFlywheelVoltage(Volts.of(FLYWHEEL_TEST_VOLTAGE));
                  elapsed[0] += 0.02;
                  if (elapsed[0] > WARMUP_S) {
                    double vel = Math.abs(shooter.inputs.flywheelVelocity.in(RotationsPerSecond));
                    recordSample(step, "Flywheel", vel);
                    Logger.recordOutput("TestMode/LiveVelocity/Flywheel", vel);
                  }
                },
                shooter)
            .withTimeout(STEP_DURATION_S),
        Commands.runOnce(() -> shooter.io.setFlywheelVoltage(Volts.of(0)), shooter),
        Commands.runOnce(() -> finalizeStep(step, keys)),
        Commands.waitSeconds(STOP_GAP_S));
  }

  private Command feederStep(Shooter shooter) {
    final String step = "SHOOTER_FEEDER";
    final String[] keys = {"Feeder"};
    final double[] elapsed = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> announceStep(step)),
        Commands.run(
                () -> {
                  shooter.io.setFeederVoltage(Volts.of(FEEDER_TEST_VOLTAGE));
                  elapsed[0] += 0.02;
                  if (elapsed[0] > WARMUP_S) {
                    double vel = Math.abs(shooter.inputs.feederVelocity.in(RotationsPerSecond));
                    recordSample(step, "Feeder", vel);
                    Logger.recordOutput("TestMode/LiveVelocity/Feeder", vel);
                  }
                },
                shooter)
            .withTimeout(SHORT_STEP_DURATION_S),
        Commands.runOnce(() -> shooter.io.setFeederVoltage(Volts.of(0)), shooter),
        Commands.runOnce(() -> finalizeStep(step, keys)),
        Commands.waitSeconds(STOP_GAP_S));
  }

  private Command hopperStep(Hopper hopper) {
    final String step = "HOPPER_INDEXER";
    final String[] keys =
        HopperConstants.USE_TOP_INDEXER
            ? new String[] {"Indexer", "TopIndexer"}
            : new String[] {"Indexer"};
    final double[] elapsed = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> announceStep(step)),
        Commands.run(
                () -> {
                  hopper.io.setIndexerVoltage(Volts.of(INDEXER_TEST_VOLTAGE));
                  if (HopperConstants.USE_TOP_INDEXER) {
                    hopper.io.setTopIndexerVoltage(Volts.of(INDEXER_TEST_VOLTAGE));
                  }
                  elapsed[0] += 0.02;
                  if (elapsed[0] > WARMUP_S) {
                    double vel = Math.abs(hopper.inputs.indexerVelocity.in(RotationsPerSecond));
                    recordSample(step, "Indexer", vel);
                    Logger.recordOutput("TestMode/LiveVelocity/Indexer", vel);
                    if (HopperConstants.USE_TOP_INDEXER) {
                      double topVel =
                          Math.abs(hopper.inputs.topIndexerVelocity.in(RotationsPerSecond));
                      recordSample(step, "TopIndexer", topVel);
                      Logger.recordOutput("TestMode/LiveVelocity/TopIndexer", topVel);
                    }
                  }
                },
                hopper)
            .withTimeout(SHORT_STEP_DURATION_S),
        Commands.runOnce(
            () -> {
              hopper.io.setIndexerVoltage(Volts.of(0));
              if (HopperConstants.USE_TOP_INDEXER) hopper.io.setTopIndexerVoltage(Volts.of(0));
            },
            hopper),
        Commands.runOnce(() -> finalizeStep(step, keys)),
        Commands.waitSeconds(STOP_GAP_S));
  }

  private Command rollerStep(IntakeRoller intakeRoller) {
    final String step = "INTAKE_ROLLER";
    final String[] keys = {"Roller"};
    final double[] elapsed = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> announceStep(step)),
        // Zero desiredIntakeVelocity first to prevent jam detection from triggering
        intakeRoller.stopIntake(),
        Commands.run(
                () -> {
                  intakeRoller.io.setIntakeVoltage(Volts.of(ROLLER_TEST_VOLTAGE));
                  elapsed[0] += 0.02;
                  if (elapsed[0] > WARMUP_S) {
                    double vel =
                        Math.abs(intakeRoller.inputs.intakeVelocity.in(RotationsPerSecond));
                    recordSample(step, "Roller", vel);
                    Logger.recordOutput("TestMode/LiveVelocity/Roller", vel);
                  }
                },
                intakeRoller)
            .withTimeout(SHORT_STEP_DURATION_S),
        intakeRoller.stopIntake(),
        Commands.runOnce(() -> finalizeStep(step, keys)),
        Commands.waitSeconds(STOP_GAP_S));
  }

  private Command pivotStep(IntakePivot intakePivot) {
    final String stepUp = "INTAKE_PIVOT_UP";
    final String stepDown = "INTAKE_PIVOT_DOWN";
    final String key = "Pivot";

    return Commands.sequence(
        // Move to INTAKE_DOWN using position control as a starting point
        Commands.runOnce(
            () -> {
              announceStep("INTAKE_PIVOT_SETUP");
              intakePivot.io.setIntakePosition(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
            },
            intakePivot),
        Commands.waitSeconds(PIVOT_SETUP_WAIT_S),

        // Up sweep: constant voltage from INTAKE_DOWN to INTAKE_UP, collect velocity
        Commands.runOnce(() -> announceStep(stepUp)),
        Commands.run(
                () -> {
                  intakePivot.io.setIntakeArmVoltage(Volts.of(PIVOT_TEST_VOLTAGE));
                  double vel =
                      Math.abs(intakePivot.inputs.intakeArmVelocity.in(RotationsPerSecond));
                  recordSample(stepUp, key, vel);
                  Logger.recordOutput("TestMode/LiveVelocity/Pivot", vel);
                },
                intakePivot)
            .until(
                () ->
                    intakePivot.inputs.intakeArmPosition.in(Degrees)
                        >= IntakeConstants.INTAKE_UP_VALUE.get())
            .withTimeout(PIVOT_SWEEP_TIMEOUT_S),
        Commands.runOnce(
            () -> {
              intakePivot.io.setIntakeArmVoltage(Volts.of(0));
              finalizeStep(stepUp, new String[] {key});
            },
            intakePivot),
        Commands.waitSeconds(0.3),

        // Down sweep: constant negative voltage from INTAKE_UP to INTAKE_DOWN
        Commands.runOnce(() -> announceStep(stepDown)),
        Commands.run(
                () -> {
                  intakePivot.io.setIntakeArmVoltage(Volts.of(-PIVOT_TEST_VOLTAGE));
                  double vel =
                      Math.abs(intakePivot.inputs.intakeArmVelocity.in(RotationsPerSecond));
                  recordSample(stepDown, key, vel);
                  Logger.recordOutput("TestMode/LiveVelocity/Pivot", vel);
                },
                intakePivot)
            .until(
                () ->
                    intakePivot.inputs.intakeArmPosition.in(Degrees)
                        <= IntakeConstants.INTAKE_DOWN_VALUE.get())
            .withTimeout(PIVOT_SWEEP_TIMEOUT_S)
            .finallyDo(
                () -> {
                  intakePivot.io.setIntakeArmVoltage(Volts.of(0));
                  finalizeStep(stepDown, new String[] {key});
                }),
        Commands.waitSeconds(STOP_GAP_S));
  }

  // ── Internal helpers ─────────────────────────────────────────────────────

  private void recordSample(String step, String motorKey, double velocity) {
    allSamples.computeIfAbsent(step + "/" + motorKey, k -> new ArrayList<>()).add(velocity);
  }

  private void announceStep(String step) {
    Logger.recordOutput("TestMode/ActiveStep", step);
    System.out.println("[TestMode] >>> " + step);
    DriverStation.reportWarning("[TestMode] " + step, false);
  }

  private void finalizeStep(String step, String[] motorKeys) {
    for (String key : motorKeys) {
      List<Double> samples = allSamples.getOrDefault(step + "/" + key, List.of());
      double mean =
          samples.isEmpty()
              ? 0.0
              : samples.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
      double min =
          samples.isEmpty()
              ? 0.0
              : samples.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);
      double baselineMean = getBaselineProp(step, key, "mean");
      double baselineMin = getBaselineProp(step, key, "min");
      TestStatus status = computeStatus(mean, baselineMean);
      results.add(new MotorResult(step, key, mean, min, baselineMean, baselineMin, status));
      System.out.printf(
          "[TestMode]   %s/%s: mean=%.3f  min=%.3f  baseline=%.3f → %s%n",
          step, key, mean, min, baselineMean, status);
    }
  }

  private TestStatus computeStatus(double measured, double baselineMean) {
    if (Double.isNaN(baselineMean) || baselineMean <= 0) return TestStatus.NO_BASELINE;
    double ratio = measured / baselineMean;
    if (ratio < FAIL_BELOW) return TestStatus.FAIL;
    if (ratio < WARN_BELOW) return TestStatus.WARN;
    if (ratio > WARN_ABOVE) return TestStatus.WARN;
    return TestStatus.PASS;
  }

  private double getBaselineProp(String step, String motorKey, String metric) {
    String propKey = step + "." + motorKey + "." + metric;
    String val = baseline.getProperty(propKey);
    if (val == null) return Double.NaN;
    try {
      return Double.parseDouble(val);
    } catch (NumberFormatException e) {
      return Double.NaN;
    }
  }

  private boolean hasBaseline() {
    return new File(BASELINE_PATH).exists() && !baseline.isEmpty();
  }

  private void loadBaseline() {
    File f = new File(BASELINE_PATH);
    if (!f.exists()) return;
    try (FileInputStream fis = new FileInputStream(f)) {
      baseline.load(fis);
      System.out.println("[TestMode] Baseline loaded from " + BASELINE_PATH);
    } catch (IOException e) {
      System.err.println("[TestMode] Could not load baseline: " + e.getMessage());
    }
  }

  private void saveBaseline() {
    Properties newBaseline = new Properties();
    for (MotorResult r : results) {
      String prefix = r.step() + "." + r.motorKey();
      newBaseline.setProperty(prefix + ".mean", String.format("%.5f", r.measuredMean()));
      newBaseline.setProperty(prefix + ".min", String.format("%.5f", r.measuredMin()));
    }
    DateTimeFormatter fmt = DateTimeFormatter.ofPattern("yyyy-MM-dd'T'HH:mm:ss");
    String comment = "Baseline saved " + LocalDateTime.now().format(fmt);
    try (FileOutputStream fos = new FileOutputStream(BASELINE_PATH)) {
      newBaseline.store(fos, comment);
      System.out.println("[TestMode] Baseline saved to " + BASELINE_PATH);
    } catch (IOException e) {
      System.err.println("[TestMode] Could not save baseline: " + e.getMessage());
    }
    baseline = newBaseline;
  }

  // ── Data types ───────────────────────────────────────────────────────────

  public enum TestStatus {
    PASS,
    WARN,
    FAIL,
    NO_BASELINE
  }

  public record MotorResult(
      String step,
      String motorKey,
      double measuredMean,
      double measuredMin,
      double baselineMean,
      double baselineMin,
      TestStatus status) {}
}
