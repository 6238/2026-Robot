package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
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

    public record EncoderTestProfile(
            String motorName,
            String encoderName,
            Voltage voltage) {
    }

    public static boolean withinThreshold(double value, double target, double threshold) {
        if (value >= target - threshold && value <= target + threshold) {
            return true;
        }
        return false;
    }

    public static boolean belowThreshold(double value, double target, double threshold) {
        if (value >= target - threshold) {
            return false;
        }
        return true;
    }

    public static boolean aboveThreshold(double value, double target, double threshold) {
        if (value <= target + threshold) {
            return false;
        }
        return true;
    }

    public static Command testLimitedMotorOutputProfile(Subsystem subsystem, Consumer<TestResult> results,
            LimitedMotorTestProfile testProfile,
            Consumer<Voltage> motorVoltageConsumer, Consumer<Angle> motorPositionConsumer,
            Supplier<Current> motorCurrentSupplier,
            Supplier<AngularVelocity> motorVelocitySupplier, Supplier<Angle> motorPositionSupplier) {

        TimedSampleBuffer<Current> currentSampleBuffer = new TimedSampleBuffer<Current>();

        AtomicBoolean endedFlag = new AtomicBoolean(false);

        return Commands.sequence(
                Commands.runOnce(() -> motorPositionConsumer.accept(testProfile.maxAngle), subsystem), // Test Max
                                                                                                       // Position

                Commands.waitTime(testProfile.maxSettleTime).andThen(
                        Commands.runOnce(() -> endedFlag.set(true)))
                        .until(() -> withinThreshold(motorPositionSupplier.get().in(Rotations),
                                testProfile.maxAngle.in(Rotations), testProfile.angleThreshold.in(Rotations))), // Wait
                                                                                                                // for
                                                                                                                // motor
                                                                                                                // to
                                                                                                                // settle

                Commands.either(

                        Commands.runOnce(() -> results.accept(new TestResult(false,
                                "Motor " + testProfile.motorName + " failed to reach max angle: wanted: "
                                        + testProfile.maxAngle.in(Degrees) + " degrees got: "
                                        + motorPositionSupplier.get().in(Degrees) + " degrees"))),

                        Commands.sequence(
                                Commands.runOnce(() -> motorPositionConsumer.accept(testProfile.minAngle), subsystem), // Test
                                                                                                                       // Min
                                                                                                                       // Position

                                Commands.waitTime(testProfile.maxSettleTime).andThen(
                                        Commands.runOnce(() -> endedFlag.set(true)))
                                        .until(() -> withinThreshold(motorPositionSupplier.get().in(Rotations),
                                                testProfile.minAngle.in(Rotations),
                                                testProfile.angleThreshold.in(Rotations))), // Wait for motor to settle

                                Commands.either(
                                        Commands.runOnce(() -> results.accept(new TestResult(false,
                                                "Motor " + testProfile.motorName
                                                        + " failed to reach min angle: wanted: "
                                                        + testProfile.minAngle.in(Degrees) + " degrees got: "
                                                        + motorPositionSupplier.get().in(Degrees) + " degrees"))),

                                        Commands.sequence(

                                                Commands.runOnce(() -> motorVoltageConsumer.accept(testProfile.voltage),
                                                        subsystem),
                                                Commands.run(() -> {
                                                    currentSampleBuffer.add(
                                                            Time.ofBaseUnits(Timer.getTimestamp(), Seconds),
                                                            motorCurrentSupplier.get());
                                                }).withTimeout(testProfile.duration.in(Seconds)),
                                                Commands.runOnce(() -> motorVoltageConsumer.accept(Volts.of(0)),
                                                        subsystem),

                                                Commands.runOnce(() -> {
                                                    // Check test results
                                                    List<TimedSample<Current>> currentSamples = currentSampleBuffer
                                                            .getLast(testProfile.sampleTime);
                                                    OptionalDouble averageCurrent = currentSamples.stream()
                                                            .mapToDouble((sample) -> sample.value().in(Amps)).average();
                                                    if (averageCurrent.isEmpty()) {
                                                        results.accept(
                                                                new TestResult(false, "Motor " + testProfile.motorName
                                                                        + " current samples not available"));
                                                        return;
                                                    }

                                                    if (withinThreshold(averageCurrent.getAsDouble(),
                                                            testProfile.targetCurrent.in(Amps),
                                                            testProfile.currentThreshold.in(Amps))) {
                                                        results.accept(new TestResult(true,
                                                                "Motor " + testProfile.motorName
                                                                        + " current draw within limits: "
                                                                        + averageCurrent.getAsDouble()
                                                                        + "A (threshold: "
                                                                        + testProfile.currentThreshold.in(Amps)
                                                                        + "A)"));
                                                        return;
                                                    }

                                                    if (aboveThreshold(averageCurrent.getAsDouble(),
                                                            testProfile.targetCurrent.in(Amps),
                                                            testProfile.currentThreshold.in(Amps))) {
                                                        results.accept(new TestResult(false,
                                                                "Motor " + testProfile.motorName
                                                                        + " drew too much current: "
                                                                        + averageCurrent.getAsDouble()
                                                                        + "A (target: " + testProfile.targetCurrent
                                                                        + "A)"));
                                                        return;
                                                    }

                                                    results.accept(new TestResult(false,
                                                            "Motor " + testProfile.motorName
                                                                    + " drew too little current: "
                                                                    + averageCurrent.getAsDouble()
                                                                    + "A (target: " + testProfile.targetCurrent
                                                                    + "A)"));
                                                })

                                        ),
                                        () -> endedFlag.get())),
                        () -> endedFlag.get()));
    }

    // A command for testing the output and freedom of movement of a motor
    // If min and max angle are supplied then the test will check that the motor can
    // reach those angles
    // Ensure actuator software limits are enabled before running this test on a
    // angle limited actuator
    public static Command testMotorOutputProfile(Subsystem subsystem, Consumer<TestResult> results,
            MotorTestProfile testProfile,
            Consumer<Voltage> motorVoltageConsumer, Supplier<Current> motorCurrentSupplier,
            Supplier<AngularVelocity> motorVelocitySupplier) {

        TimedSampleBuffer<Current> currentSampleBuffer = new TimedSampleBuffer<Current>();
        // Test a freespinning actuator
        return Commands.sequence(
                Commands.runOnce(() -> motorVoltageConsumer.accept(testProfile.voltage), subsystem),
                Commands.run(() -> {
                    currentSampleBuffer.add(Time.ofBaseUnits(Timer.getTimestamp(), Seconds),
                            motorCurrentSupplier.get());
                }).withTimeout(testProfile.duration.in(Seconds)),
                Commands.runOnce(() -> motorVoltageConsumer.accept(Volts.of(0)), subsystem),
                Commands.runOnce(() -> {
                    // Check test results
                    List<TimedSample<Current>> currentSamples = currentSampleBuffer.getLast(testProfile.sampleTime);
                    OptionalDouble averageCurrent = currentSamples.stream()
                            .mapToDouble((sample) -> sample.value().in(Amps)).average();
                    if (averageCurrent.isEmpty()) {
                        results.accept(new TestResult(false,
                                "Motor " + testProfile.motorName + " current samples not available"));
                        return;
                    }

                    if (withinThreshold(averageCurrent.getAsDouble(), testProfile.targetCurrent.in(Amps),
                            testProfile.currentThreshold.in(Amps))) {
                        results.accept(new TestResult(true,
                                "Motor " + testProfile.motorName + " current draw within limits: "
                                        + averageCurrent.getAsDouble()
                                        + "A (threshold: " + testProfile.currentThreshold.in(Amps) + "A)"));
                        return;
                    }

                    if (aboveThreshold(averageCurrent.getAsDouble(), testProfile.targetCurrent.in(Amps),
                            testProfile.currentThreshold.in(Amps))) {
                        results.accept(new TestResult(false,
                                "Motor " + testProfile.motorName + " drew too much current: "
                                        + averageCurrent.getAsDouble()
                                        + "A (target: " + testProfile.targetCurrent + "A)"));
                        return;
                    }

                    results.accept(new TestResult(false,
                            "Motor " + testProfile.motorName + " drew too little current: "
                                    + averageCurrent.getAsDouble()
                                    + "A (target: " + testProfile.targetCurrent + "A)"));
                }));
    }

    public record TestResult(
            Boolean success,
            String message) {
    }

    public static class TestResultAggregator {
        public List<TestResult> results;

        public TestResultAggregator() {
            results = new ArrayList<TestResult>();
        }

        public Consumer<TestResult> getTestResultConsumer() {
            return (TestResult result) -> {
                results.add(result);
            };
        }

        public Command outputResults() {
            return Commands.runOnce(() -> {
                DataLogManager.log(
                        results.stream().map((val) -> (val.success ? "PASS" : "FAIL") + val.message + "\n").reduce("",
                                (a, b) -> a + b));
            });
        }
    }
}
