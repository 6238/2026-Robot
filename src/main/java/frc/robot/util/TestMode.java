package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.util.TimedSampleBuffer.TimedSample;

public class TestMode {

    public record MotorTestProfile(
            String motorName,
            Voltage voltage,
            Time duration,
            Time sampleTime,
            AngularVelocity targetVelocity,
            Current targetCurrent,
            Current currentThreshold) {
    }

    public record LimitedMotorTestProfile(
            String motorName,
            Voltage voltage,
            Time duration,
            Time sampleTime,
            AngularVelocity targetVelocity,
            Current targetCurrent,
            Current currentThreshold,
            Angle minAngle,
            Angle maxAngle,
            Angle angleThreshold,
            Time maxSettleTime) {
    }

    public static boolean withinThreshold(double value, double target, double threshold) {
        return (value >= target - threshold && value <= target + threshold);
    }

    public static boolean belowThreshold(double value, double target, double threshold) {
        return !(value >= target - threshold);
    }

    public static boolean aboveThreshold(double value, double target, double threshold) {
        return !(value <= target + threshold);
    }

    public static final class NTTestHandle {
        public final String testId;
        private final String name;
        private final Double durationSec;

        private final StringPublisher namePub;
        private final DoublePublisher durationPub;
        private final DoublePublisher progressPub;
        private final StringPublisher statusPub;     // "PENDING" | "RUNNING" | "PASS" | "FAIL"
        private final BooleanPublisher donePub;
        private final BooleanPublisher passPub;
        private final StringPublisher messagePub;

        private final double startTime;

        private NTTestHandle(NetworkTable table, String testId, String name, double durationSec) {
            this.testId = testId;
            this.name = name;
            this.durationSec = durationSec;

            NetworkTable t = table.getSubTable(testId);

            namePub = t.getStringTopic("name").publish();
            durationPub = t.getDoubleTopic("durationSec").publish();
            progressPub = t.getDoubleTopic("progress").publish();
            statusPub = t.getStringTopic("status").publish();
            donePub = t.getBooleanTopic("done").publish();
            passPub = t.getBooleanTopic("pass").publish();
            messagePub = t.getStringTopic("message").publish();

            startTime = Timer.getFPGATimestamp();
            namePub.set(name);
            durationPub.set(durationSec);
            progressPub.set(0.0);
            statusPub.set("PENDING");
            donePub.set(false);
            passPub.set(false);
            messagePub.set("");
        }

        public String getName() {
            return name;
        }

        public void setRunning() {
            statusPub.set("RUNNING");
            donePub.set(false);
        }

        public void updateProgressByTime(double elapsedSec) {
            double p = durationSec <= 1e-9 ? 0.0 : clamp01(elapsedSec / durationSec);
            progressPub.set(p);
        }

        public void setProgress(double p) {
            progressPub.set(clamp01(p));
        }

        public void pass(String msg) {
            statusPub.set("PASS");
            donePub.set(true);
            passPub.set(true);
            progressPub.set(1.0);
            messagePub.set(msg == null ? "" : msg);
        }

        public void fail(String msg) {
            statusPub.set("FAIL");
            donePub.set(true);
            passPub.set(false);
            progressPub.set(1.0);
            messagePub.set(msg == null ? "" : msg);
        }

        public double secondsSinceCreate() {
            return Timer.getFPGATimestamp() - startTime;
        }

        private static double clamp01(double v) {
            if (v < 0) return 0;
            if (v > 1) return 1;
            return v;
        }
    }

    public static NTTestHandle createAndPublishTest(String testId, String testName, double durationSec) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable root = inst.getTable("testmode").getSubTable("tests");
        return new NTTestHandle(root, testId, testName, durationSec);
    }

    public static Command testMotorOutputProfile(
            Subsystem subsystem,
            NTTestHandle test,
            MotorTestProfile testProfile,
            Consumer<Voltage> motorVoltageConsumer,
            Supplier<Current> motorCurrentSupplier,
            Supplier<AngularVelocity> motorVelocitySupplier) {

        TimedSampleBuffer<Current> currentSampleBuffer = new TimedSampleBuffer<>();

        AtomicReference<Double> startTime = new AtomicReference<>(0.0);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    test.setRunning();
                    startTime.set(Timer.getFPGATimestamp());
                    test.setProgress(0.0);
                    motorVoltageConsumer.accept(testProfile.voltage);
                }, subsystem),

                Commands.run(() -> {
                    double now = Timer.getFPGATimestamp();
                    double elapsed = now - startTime.get();
                    test.updateProgressByTime(elapsed);

                    currentSampleBuffer.add(Time.ofBaseUnits(Timer.getTimestamp(), Seconds),
                            motorCurrentSupplier.get());
                }).withTimeout(testProfile.duration.in(Seconds)),

                Commands.runOnce(() -> motorVoltageConsumer.accept(Volts.of(0)), subsystem),

                Commands.runOnce(() -> {
                    // Check test results
                    var currentSamples = currentSampleBuffer.getLast(testProfile.sampleTime);
                    OptionalDouble avgA = currentSamples.stream()
                            .mapToDouble((TimedSample<Current> s) -> s.value().in(Amps))
                            .average();

                    if (avgA.isEmpty()) {
                        test.fail("Current samples not available");
                        return;
                    }

                    double avg = avgA.getAsDouble();
                    double target = testProfile.targetCurrent.in(Amps);
                    double thr = testProfile.currentThreshold.in(Amps);

                    if (withinThreshold(avg, target, thr)) {
                        test.pass("Current within limits: " + avg + "A (±" + thr + "A)");
                    } else if (aboveThreshold(avg, target, thr)) {
                        test.fail("Drew too much current: " + avg + "A (target " + target + "A)");
                    } else {
                        test.fail("Drew too little current: " + avg + "A (target " + target + "A)");
                    }
                })
        );
    }

    public static Command testLimitedMotorOutputProfile(
            Subsystem subsystem,
            NTTestHandle test,
            LimitedMotorTestProfile testProfile,
            Consumer<Voltage> motorVoltageConsumer,
            Consumer<Angle> motorPositionConsumer,
            Supplier<Current> motorCurrentSupplier,
            Supplier<AngularVelocity> motorVelocitySupplier,
            Supplier<Angle> motorPositionSupplier) {

        TimedSampleBuffer<Current> currentSampleBuffer = new TimedSampleBuffer<>();
        AtomicBoolean endedFlag = new AtomicBoolean(false);

        // Progress: 0-0.33 max settle, 0.33-0.66 min settle, 0.66-1.0 current test
        AtomicReference<Double> phaseStart = new AtomicReference<>(0.0);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    test.setRunning();
                    test.setProgress(0.0);
                    endedFlag.set(false);
                    phaseStart.set(Timer.getFPGATimestamp());
                    motorPositionConsumer.accept(testProfile.maxAngle);
                }, subsystem),

                Commands.waitTime(testProfile.maxSettleTime).andThen(Commands.runOnce(() -> endedFlag.set(true)))
                        .until(() -> withinThreshold(
                                motorPositionSupplier.get().in(Rotations),
                                testProfile.maxAngle.in(Rotations),
                                testProfile.angleThreshold.in(Rotations)))
                        .finallyDo(() -> test.setProgress(0.33)),

                Commands.either(
                        Commands.runOnce(() -> test.fail(
                                "Failed to reach max angle; wanted " + testProfile.maxAngle.in(Degrees) +
                                        " deg got " + motorPositionSupplier.get().in(Degrees) + " deg")),
                        Commands.sequence(
                                Commands.runOnce(() -> {
                                    endedFlag.set(false);
                                    phaseStart.set(Timer.getFPGATimestamp());
                                    motorPositionConsumer.accept(testProfile.minAngle);
                                }, subsystem),

                                Commands.waitTime(testProfile.maxSettleTime).andThen(Commands.runOnce(() -> endedFlag.set(true)))
                                        .until(() -> withinThreshold(
                                                motorPositionSupplier.get().in(Rotations),
                                                testProfile.minAngle.in(Rotations),
                                                testProfile.angleThreshold.in(Rotations)))
                                        .finallyDo(() -> test.setProgress(0.66)),

                                Commands.either(
                                        Commands.runOnce(() -> test.fail(
                                                "Failed to reach min angle; wanted " + testProfile.minAngle.in(Degrees) +
                                                        " deg got " + motorPositionSupplier.get().in(Degrees) + " deg")),
                                        Commands.sequence(
                                                Commands.runOnce(() -> motorVoltageConsumer.accept(testProfile.voltage), subsystem),
                                                Commands.run(() -> {
                                                    currentSampleBuffer.add(
                                                            Time.ofBaseUnits(Timer.getTimestamp(), Seconds),
                                                            motorCurrentSupplier.get());
                                                }).withTimeout(testProfile.duration.in(Seconds)),
                                                Commands.runOnce(() -> motorVoltageConsumer.accept(Volts.of(0)), subsystem),
                                                Commands.runOnce(() -> {
                                                    var currentSamples = currentSampleBuffer.getLast(testProfile.sampleTime);
                                                    OptionalDouble avgA = currentSamples.stream()
                                                            .mapToDouble(s -> s.value().in(Amps))
                                                            .average();
                                                    if (avgA.isEmpty()) {
                                                        test.fail("Current samples not available");
                                                        return;
                                                    }

                                                    double avg = avgA.getAsDouble();
                                                    double target = testProfile.targetCurrent.in(Amps);
                                                    double thr = testProfile.currentThreshold.in(Amps);

                                                    if (withinThreshold(avg, target, thr)) {
                                                        test.pass("Current within limits: " + avg + "A (±" + thr + "A)");
                                                    } else if (aboveThreshold(avg, target, thr)) {
                                                        test.fail("Drew too much current: " + avg + "A (target " + target + "A)");
                                                    } else {
                                                        test.fail("Drew too little current: " + avg + "A (target " + target + "A)");
                                                    }
                                                }).beforeStarting(() -> test.setProgress(0.66))
                                                  .finallyDo(() -> test.setProgress(1.0))
                                        ),
                                        () -> endedFlag.get()
                                )
                        ),
                        () -> endedFlag.get()
                )
        );
    }
}
