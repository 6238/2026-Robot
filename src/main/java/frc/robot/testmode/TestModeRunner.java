package frc.robot.testmode;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
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
import java.util.function.Consumer;
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
  static final double WARN_BELOW = 0.85; // WARN if measured < 75% of baseline
  static final double FAIL_BELOW = 0.75; // FAIL if measured < 50% of baseline
  static final double WARN_ABOVE = 1.25; // WARN if measured > 130% of baseline

  // ── Persistence ─────────────────────────────────────────────────────────
  static final String BASELINE_PATH = "/home/lvuser/test-baseline.properties";

  // ── Instance state (fresh per test run) ─────────────────────────────────
  private final Map<String, List<Double>> allSamples = new LinkedHashMap<>();
  private final List<MotorResult> results = new ArrayList<>();
  private Properties baseline = new Properties();

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
      IntakePivot intakePivot,
      boolean baselineMode,
      Consumer<Boolean> onComplete) {

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (!baselineMode) loadBaseline();
                  Logger.recordOutput("TestMode/BaselineMode", baselineMode);
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
            pivotStep(intakePivot, intakeRoller),

            // Finalize
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "COMPLETE");
                  saveBaseline();
                  String html =
                      TestModeReport.generate(
                          results,
                          RobotController.getSerialNumber(),
                          BuildConstants.GIT_SHA,
                          baselineMode);
                  TestModeReport.writeToDisk(html);
                  TestModeHttpServer.serveReport(html);
                  if (baselineMode) {
                    onComplete.accept(true);
                  } else {
                    boolean anyFailure =
                        results.stream()
                            .anyMatch(
                                r ->
                                    r.status() == TestStatus.FAIL || r.status() == TestStatus.WARN);
                    onComplete.accept(!anyFailure);
                  }
                  System.out.println("[TestMode] === Test Sequence Complete ===");
                  DriverStation.reportWarning(
                      "[TestMode] Complete — check HTML report on USB drive", false);
                }))
        .withName("Robot Test Sequence");
  }

  /**
   * Ball Path mode: actuates each mechanism in ball-path order without recording data or comparing
   * baselines. Used to visually verify the full intake→shoot path.
   */
  public Command buildBallPathSequence(
      Shooter shooter, Hopper hopper, IntakeRoller intakeRoller, IntakePivot intakePivot) {
    double partialAngle =
        (IntakeConstants.INTAKE_DOWN_VALUE.get() + IntakeConstants.INTAKE_UP_VALUE.get() + 30)
            / 2.0;

    return Commands.sequence(
            // 1. Intake pivot down
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "BALL_PATH_PIVOT_DOWN");
                  System.out.println("[BallPath] Step 1: Pivot down");
                }),
            intakePivot.setIntakeAngle(() -> Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get())),
            Commands.waitSeconds(PIVOT_SETUP_WAIT_S),

            // 2. Intake roller
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "BALL_PATH_ROLLER");
                  System.out.println("[BallPath] Step 2: Intake roller");
                }),
            intakeRoller.spinIntake(),
            Commands.waitSeconds(SHORT_STEP_DURATION_S),
            intakeRoller.stopIntake(),
            Commands.waitSeconds(STOP_GAP_S),

            // 3. Intake pivot partially up
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "BALL_PATH_PIVOT_PARTIAL");
                  System.out.println("[BallPath] Step 3: Pivot partial");
                }),
            intakePivot.setIntakeAngle(() -> Degrees.of(partialAngle)),
            Commands.waitSeconds(PIVOT_SETUP_WAIT_S),

            // 4. Shooter flywheel 10 RPS
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "BALL_PATH_FLYWHEEL");
                  System.out.println("[BallPath] Step 4: Flywheel 4 Volts");
                  shooter.setFlywheelVoltage(Volts.of(4));
                },
                shooter),
            Commands.waitSeconds(SHORT_STEP_DURATION_S),

            // 5. Feeder 10 RPS
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "BALL_PATH_FEEDER");
                  System.out.println("[BallPath] Step 5: Feeder 4 Volts");
                  shooter.setFeederVoltage(Volts.of(4));
                },
                shooter),
            Commands.waitSeconds(SHORT_STEP_DURATION_S),

            // 6. Indexer
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "BALL_PATH_INDEXER");
                  System.out.println("[BallPath] Step 6: Indexer");
                }),
            hopper.spinFullIndexer(),
            Commands.waitSeconds(SHORT_STEP_DURATION_S),

            // Stop everything
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("TestMode/ActiveStep", "COMPLETE");
                  shooter.setFlywheelVoltage(Volts.of(0));
                  shooter.setFeederVoltage(Volts.of(0));
                },
                shooter),
            hopper.stopFullIndexer())
        .withName("Ball Path Sequence");
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
                  hopper.io.setIndexerSpeed(
                      RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get()));
                  if (HopperConstants.USE_TOP_INDEXER) {
                    hopper.io.setTopIndexerSpeed(
                        RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get()));
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
              hopper.io.setIndexerSpeed(RotationsPerSecond.of(0));
              if (HopperConstants.USE_TOP_INDEXER)
                hopper.io.setTopIndexerSpeed(RotationsPerSecond.of(0));
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

  private Command pivotStep(IntakePivot intakePivot, IntakeRoller intakeRoller) {
    final String stepUp = "INTAKE_PIVOT_UP";
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

        // Test roller now that pivot is at INTAKE_DOWN
        rollerStep(intakeRoller),

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
