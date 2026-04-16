# Drive Responsiveness Improvements Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the drivebase more responsive for defense play by fixing loop overrun lock contention, raising the setpoint generator steer rate limit, replacing the stick curve with Betaflight's Actual rate model, adding a setpoint generator bypass flag, and adding a Y-button defense mode that raises drive current and lowers other subsystem current.

**Architecture:** Each improvement is isolated — `DriveConstants` for numeric knobs, `Drive`/`Module` restructure for lock contention, `DriveCommands` for stick curves, IO interface + TalonFX impl pattern (matching all existing subsystems) for defense mode current reconfiguration.

**Tech Stack:** WPILib Command-based Java, Phoenix 6 TalonFX, AdvantageKit (AK), JUnit 5 + Mockito 5.

---

## File Map

| File | Change |
|---|---|
| `src/main/java/frc/robot/subsystems/drive/DriveConstants.java` | Add `BYPASS_SETPOINT_GENERATOR` flag; raise `MAX_MODULE_ANGULAR_VELOCITY` 30→50 |
| `src/main/java/frc/robot/subsystems/drive/Drive.java` | Bypass path in `runVelocity`; `setDefenseMode`; move Logger calls outside lock |
| `src/main/java/frc/robot/subsystems/drive/Module.java` | Add `updateHardwareInputs()` and `logAndProcessInputs()`; keep `periodic()` as delegate |
| `src/main/java/frc/robot/subsystems/drive/ModuleIO.java` | Add `setDriveCurrentLimits` default no-op |
| `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java` | Implement `setDriveCurrentLimits` |
| `src/main/java/frc/robot/commands/DriveCommands.java` | Replace quadratic with Betaflight Actual rate model |
| `src/main/java/frc/robot/subsystems/shooter/ShooterIO.java` | Add `setDefenseMode` default no-op |
| `src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java` | Implement `setDefenseMode` |
| `src/main/java/frc/robot/subsystems/shooter/Shooter.java` | Add `setDefenseMode` passthrough |
| `src/main/java/frc/robot/subsystems/hopper/HopperIO.java` | Add `setDefenseMode` default no-op |
| `src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java` | Implement `setDefenseMode` |
| `src/main/java/frc/robot/subsystems/hopper/Hopper.java` | Add `setDefenseMode` passthrough |
| `src/main/java/frc/robot/subsystems/intake/IntakeRollerIO.java` | Add `setDefenseMode` default no-op |
| `src/main/java/frc/robot/subsystems/intake/IntakeRollerIOTalonFX.java` | Implement `setDefenseMode` |
| `src/main/java/frc/robot/subsystems/intake/IntakeRoller.java` | Add `setDefenseMode` passthrough |
| `src/main/java/frc/robot/subsystems/intake/IntakePivotIO.java` | Add `setDefenseMode` default no-op |
| `src/main/java/frc/robot/subsystems/intake/IntakePivotIOTalonFX.java` | Implement `setDefenseMode` |
| `src/main/java/frc/robot/subsystems/intake/IntakePivot.java` | Add `setDefenseMode` passthrough |
| `src/main/java/frc/robot/RobotContainer.java` | Wire Y button toggle; reset defense mode in `teleopInit` |

---

## Task 1: Raise MAX_MODULE_ANGULAR_VELOCITY and Add Bypass Flag

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

- [ ] **Step 1: Update DriveConstants**

Replace the entire content of `DriveConstants.java`:

```java
package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class DriveConstants {
  /** Max steer angular velocity passed to SwerveSetpointGenerator. Raised from 30 to reduce
   * module reorientation lag (~100ms → ~31ms for 90° turn). */
  public static final AngularVelocity MAX_MODULE_ANGULAR_VELOCITY = RadiansPerSecond.of(50);

  /** When true, skips the SwerveSetpointGenerator and sends ChassisSpeeds directly to modules.
   * Eliminates setpoint lag entirely; use for testing. Auto path always uses the generator. */
  public static boolean BYPASS_SETPOINT_GENERATOR = false;
}
```

- [ ] **Step 2: Add bypass path in Drive.runVelocity**

In `Drive.java`, locate the `runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards)` method (line ~334). Replace the body with:

```java
public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
  SwerveModuleState[] setpointStates;

  if (DriveConstants.BYPASS_SETPOINT_GENERATOR) {
    setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxLinearSpeedMetersPerSec());
  } else {
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    setpointStates = previousSetpoint.moduleStates();
  }

  if (!Constants.MINIMAL_LOGGING) {
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput(
        "SwerveChassisSpeeds/Setpoints",
        DriveConstants.BYPASS_SETPOINT_GENERATOR
            ? speeds
            : previousSetpoint.robotRelativeSpeeds());
    Logger.recordOutput("Drive/BypassSetpointGenerator", DriveConstants.BYPASS_SETPOINT_GENERATOR);
  }

  double[] torqueCurrents = feedforwards.torqueCurrentsAmps();
  for (int i = 0; i < 4; i++) {
    modules[i].runSetpoint(
        new SwerveModuleState(setpointStates[i].speedMetersPerSecond, setpointStates[i].angle),
        torqueCurrents[i]);
  }

  if (!Constants.MINIMAL_LOGGING)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
}
```

Add the missing import at the top of `Drive.java`:
```java
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
```
(It may already be present — check before adding.)

- [ ] **Step 3: Build**

```bash
./gradlew build -x test
```

Expected: `BUILD SUCCESSFUL`. Fix any compile errors before continuing.

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/DriveConstants.java \
        src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "feat(drive): raise max steer velocity 30→50 rad/s, add setpoint generator bypass flag"
```

---

## Task 2: Betaflight Actual Rate Model Stick Curves

**Files:**
- Modify: `src/main/java/frc/robot/commands/DriveCommands.java`
- Test: `src/test/java/commands/DriveCommandsCurveTest.java` (new)

- [ ] **Step 1: Write the failing test**

Create `src/test/java/commands/DriveCommandsCurveTest.java`:

```java
package commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.commands.DriveCommands;
import org.junit.jupiter.api.Test;

class DriveCommandsCurveTest {

  /** Verifies the Betaflight Actual rate curve properties. */
  @Test
  void curve_atZero_returnsZero() {
    assertEquals(0.0, DriveCommands.applyCurve(0.0), 1e-9);
  }

  @Test
  void curve_atFullDeflection_returnsOne() {
    assertEquals(1.0, DriveCommands.applyCurve(1.0), 1e-9);
  }

  @Test
  void curve_atHalfDeflection_lessThanHalf() {
    // Betaflight curve is less aggressive than linear at mid-range for precise center control
    double result = DriveCommands.applyCurve(0.5);
    assertTrue(result < 0.5, "Expected curve output < 0.5 at stick=0.5, got " + result);
    assertTrue(result > 0.0, "Expected curve output > 0 at stick=0.5");
  }

  @Test
  void curve_monotonicallyIncreasing() {
    double prev = 0.0;
    for (double x = 0.1; x <= 1.0; x += 0.1) {
      double curr = DriveCommands.applyCurve(x);
      assertTrue(curr > prev, "Curve not monotonically increasing at x=" + x);
      prev = curr;
    }
  }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
./gradlew test --tests "commands.DriveCommandsCurveTest"
```

Expected: FAIL — `applyCurve` method does not exist yet.

- [ ] **Step 3: Add curve constants and applyCurve method to DriveCommands**

At the top of `DriveCommands.java`, after the existing constants (around line 50), add:

```java
// Betaflight "Actual" rate model parameters.
// output(x) = (CENTER_SENS * x + (MAX_RATE - CENTER_SENS) * x^EXPO) / MAX_RATE
// where x ∈ [0,1] is post-deadband stick magnitude and output ∈ [0,1].
public static final double CURVE_CENTER_SENS = 0.3;
public static final double CURVE_MAX_RATE = 1.0;
public static final double CURVE_EXPO = 3.0;

/**
 * Applies the Betaflight Actual rate curve to a post-deadband stick value in [0, 1].
 * Returns a value in [0, 1].
 */
public static double applyCurve(double x) {
  return (CURVE_CENTER_SENS * x + (CURVE_MAX_RATE - CURVE_CENTER_SENS) * Math.pow(x, CURVE_EXPO))
      / CURVE_MAX_RATE;
}
```

- [ ] **Step 4: Update getLinearVelocityFromJoysticks to use the curve**

Find `getLinearVelocityFromJoysticks` (line ~55). Replace the magnitude squaring line:

Before:
```java
// Square magnitude for more precise control
linearMagnitude = linearMagnitude * linearMagnitude;
```

After:
```java
// Betaflight Actual rate curve for precise center control
linearMagnitude = applyCurve(linearMagnitude);
```

- [ ] **Step 5: Update omega shaping in joystickDrive and joystickDriveRobotRelative**

In `joystickDrive` (line ~84), replace the omega squaring:

Before:
```java
// Square rotation value for more precise control
omega = Math.copySign(omega * omega, omega);
```

After:
```java
// Betaflight Actual rate curve for precise center control
omega = Math.copySign(applyCurve(Math.abs(omega)), omega);
```

Apply the same replacement in `joystickDriveRobotRelative` (same pattern, same lines).

- [ ] **Step 6: Run tests**

```bash
./gradlew test --tests "commands.DriveCommandsCurveTest"
```

Expected: all 4 tests PASS.

- [ ] **Step 7: Full test suite**

```bash
./gradlew test
```

Expected: BUILD SUCCESSFUL, no regressions.

- [ ] **Step 8: Commit**

```bash
git add src/main/java/frc/robot/commands/DriveCommands.java \
        src/test/java/commands/DriveCommandsCurveTest.java
git commit -m "feat(drive): replace quadratic stick curve with Betaflight Actual rate model"
```

---

## Task 3: Loop Overrun Fix — Reduce Odometry Lock Hold Time

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/Module.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

- [ ] **Step 1: Split Module.periodic() into two methods**

In `Module.java`, replace `periodic()` with three methods:

```java
/** Called under odometryLock — reads hardware into inputs struct. No logging. */
public void updateHardwareInputs() {
  io.updateInputs(inputs);
}

/** Called outside odometryLock — serializes inputs to AK log and updates alerts. */
public void logAndProcessInputs() {
  Logger.processInputs(logKey, inputs);

  // Calculate positions for odometry — mutate pool in-place, no per-cycle allocation
  int sampleCount = inputs.odometryTimestamps.length;
  for (int i = 0; i < sampleCount; i++) {
    odometryPositions[i].distanceMeters =
        inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
    odometryPositions[i].angle = inputs.odometryTurnPositions[i];
  }

  // Update alerts
  driveDisconnectedAlert.set(!inputs.driveConnected);
  turnDisconnectedAlert.set(!inputs.turnConnected);
  turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
}

/** Convenience delegate — preserves existing call sites in tests. */
public void periodic() {
  updateHardwareInputs();
  logAndProcessInputs();
}
```

Remove the old `periodic()` body (the original version that did everything inline). The odometry position calculation and alert updates previously in `periodic()` now live in `logAndProcessInputs()`.

- [ ] **Step 2: Restructure Drive.periodic() to release lock before logging**

In `Drive.java`, locate the `periodic()` method. Replace the lock-protected section and the immediately following Logger/module calls with:

```java
// Refresh all CAN signals in one batch before acquiring the lock.
if (allSignals.length > 0) {
  BaseStatusSignal.refreshAll(allSignals);
}

// Under lock: hardware reads only. Logger.processInputs is intentionally outside
// the lock — it reads from already-captured structs and doesn't need protection.
odometryLock.lock();
gyroIO.updateInputs(gyroInputs);
for (var module : modules) {
  module.updateHardwareInputs();
}
odometryLock.unlock();

// Outside lock: AK serialization + alert/odometry updates
Logger.processInputs("Drive/Gyro", gyroInputs);
for (var module : modules) {
  module.logAndProcessInputs();
}
```

Everything else in `periodic()` (disabled stop, odometry update loop, gyro alert, field pose) stays unchanged.

- [ ] **Step 3: Add loop timing instrumentation**

At the top of `Drive.periodic()`, before the `refreshAll` call, add:

```java
double periodicStartSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
```

At the very end of `Drive.periodic()`, add:

```java
if (!Constants.MINIMAL_LOGGING) {
  Logger.recordOutput(
      "Drive/PeriodicTimeMs",
      (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - periodicStartSec) * 1000.0);
}
```

- [ ] **Step 4: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL.

- [ ] **Step 5: Run all tests**

```bash
./gradlew test
```

Expected: all tests pass. (No Drive unit tests exist, but existing subsystem tests must not regress.)

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Module.java \
        src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "perf(drive): release odometry lock before AK Logger serialization to reduce lock contention"
```

---

## Task 4: Defense Mode — Drive Side

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIO.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

- [ ] **Step 1: Add setDriveCurrentLimits to ModuleIO interface**

In `ModuleIO.java`, add at the end before the closing `}`:

```java
/**
 * Reconfigures drive motor current limits at runtime. No-op by default (sim/test).
 *
 * @param supplyAmps supply current limit in amps
 * @param statorAmps stator current limit in amps
 */
public default void setDriveCurrentLimits(double supplyAmps, double statorAmps) {}
```

- [ ] **Step 2: Implement setDriveCurrentLimits in ModuleIOTalonFX**

In `ModuleIOTalonFX.java`, add the following import if not already present:

```java
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
```

Then add the method override before the closing `}`:

```java
@Override
public void setDriveCurrentLimits(double supplyAmps, double statorAmps) {
  var limits =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(supplyAmps)
          .withSupplyCurrentLowerLimit(supplyAmps)
          .withStatorCurrentLimit(statorAmps)
          .withStatorCurrentLimitEnable(true);
  driveTalon.getConfigurator().apply(limits);
}
```

- [ ] **Step 3: Add setDefenseMode to Drive**

In `Drive.java`, add a field after the existing `SwerveSetpoint previousSetpoint` field:

```java
private boolean defenseModeActive = false;
```

Add the method before the closing `}` of the class:

```java
/**
 * Toggles defense mode. When active, drive motor current limits are raised for pushing power;
 * other subsystems lower their limits via their own setDefenseMode() calls.
 */
public void setDefenseMode(boolean active) {
  if (active == defenseModeActive) return;
  defenseModeActive = active;
  for (var module : modules) {
    // Normal:  supply 55 A / stator 102 A (kSlipCurrent)
    // Defense: supply 80 A / stator 120 A
    module.setDriveCurrentLimits(active ? 80.0 : 55.0, active ? 120.0 : 102.0);
  }
  Logger.recordOutput("Drive/DefenseMode", defenseModeActive);
}

/** Returns whether defense mode is currently active. */
public boolean isDefenseModeActive() {
  return defenseModeActive;
}
```

- [ ] **Step 4: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL.

- [ ] **Step 5: Run tests**

```bash
./gradlew test
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/ModuleIO.java \
        src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java \
        src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "feat(drive): add defense mode — raises drive current limits via runtime TalonFX reconfiguration"
```

---

## Task 5: Defense Mode — Other Subsystems

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterIO.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/Shooter.java`
- Modify: `src/main/java/frc/robot/subsystems/hopper/HopperIO.java`
- Modify: `src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java`
- Modify: `src/main/java/frc/robot/subsystems/hopper/Hopper.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeRollerIO.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeRollerIOTalonFX.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeRoller.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakePivotIO.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakePivotIOTalonFX.java`
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakePivot.java`

**Normal current limits (for reference when writing defense values):**
- Shooter flywheel: supply 60 A / stator 130 A
- Shooter feeder: supply 50 A / stator 80 A
- Hopper indexer: stator 40 A (no supply limit enabled)
- Intake roller: supply 25 A / stator 70 A
- Intake pivot: supply 30 A / stator 40 A

- [ ] **Step 1: ShooterIO — add default**

In `ShooterIO.java`, add before the closing `}`:

```java
/** Reconfigures flywheel and feeder current limits. No-op in sim/test. */
public default void setDefenseMode(boolean active) {}
```

- [ ] **Step 2: ShooterIOTalonFX — implement**

Add import to `ShooterIOTalonFX.java` if not present:
```java
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
```

Add method before closing `}`:

```java
@Override
public void setDefenseMode(boolean active) {
  var flywheelLimits =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(active ? 30.0 : 60.0)
          .withSupplyCurrentLowerLimit(active ? 30.0 : 40.0)
          .withStatorCurrentLimit(active ? 80.0 : 130.0)
          .withStatorCurrentLimitEnable(true);
  flywheelTalon.getConfigurator().apply(flywheelLimits);
  flywheel2Talon.getConfigurator().apply(flywheelLimits);

  var feederLimits =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(active ? 25.0 : 50.0)
          .withSupplyCurrentLowerLimit(active ? 25.0 : 40.0)
          .withStatorCurrentLimit(active ? 40.0 : 80.0)
          .withStatorCurrentLimitEnable(true);
  feederTalon.getConfigurator().apply(feederLimits);
}
```

- [ ] **Step 3: Shooter — add passthrough**

In `Shooter.java`, add before the closing `}`:

```java
public void setDefenseMode(boolean active) {
  io.setDefenseMode(active);
}
```

- [ ] **Step 4: HopperIO — add default**

In `HopperIO.java`, add before the closing `}`:

```java
/** Reconfigures indexer current limits. No-op in sim/test. */
public default void setDefenseMode(boolean active) {}
```

- [ ] **Step 5: HopperIOTalonFX — implement**

Add import to `HopperIOTalonFX.java` if not present:
```java
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
```

Add method before closing `}`:

```java
@Override
public void setDefenseMode(boolean active) {
  var limits =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(active ? 20.0 : 40.0)
          .withStatorCurrentLimitEnable(true);
  indexerTalon.getConfigurator().apply(limits);
  topIndexerTalon.getConfigurator().apply(limits);
}
```

- [ ] **Step 6: Hopper — add passthrough**

In `Hopper.java`, add before the closing `}`:

```java
public void setDefenseMode(boolean active) {
  io.setDefenseMode(active);
}
```

- [ ] **Step 7: IntakeRollerIO — add default**

In `IntakeRollerIO.java`, add before the closing `}`:

```java
/** Reconfigures roller current limits. No-op in sim/test. */
public default void setDefenseMode(boolean active) {}
```

- [ ] **Step 8: IntakeRollerIOTalonFX — implement**

Add import to `IntakeRollerIOTalonFX.java` if not present:
```java
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
```

Add method before closing `}`:

```java
@Override
public void setDefenseMode(boolean active) {
  var limits =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(active ? 15.0 : 25.0)
          .withSupplyCurrentLowerLimit(active ? 15.0 : 25.0)
          .withStatorCurrentLimit(active ? 35.0 : 70.0)
          .withStatorCurrentLimitEnable(true);
  intakeTalon.getConfigurator().apply(limits);
  intakeFollowerTalon.getConfigurator().apply(limits);
}
```

- [ ] **Step 9: IntakeRoller — add passthrough**

In `IntakeRoller.java`, find the `io` field (it is `IntakeRollerIO io`). Add before the closing `}`:

```java
public void setDefenseMode(boolean active) {
  io.setDefenseMode(active);
}
```

- [ ] **Step 10: IntakePivotIO — add default**

In `IntakePivotIO.java`, add before the closing `}`:

```java
/** Reconfigures pivot arm current limits. No-op in sim/test. */
public default void setDefenseMode(boolean active) {}
```

- [ ] **Step 11: IntakePivotIOTalonFX — implement**

Add import to `IntakePivotIOTalonFX.java` if not present:
```java
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
```

Add method before closing `}`:

```java
@Override
public void setDefenseMode(boolean active) {
  var limits =
      new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(active ? 15.0 : 30.0)
          .withSupplyCurrentLowerLimit(active ? 15.0 : 20.0)
          .withStatorCurrentLimit(active ? 20.0 : 40.0)
          .withStatorCurrentLimitEnable(true);
  intakeArmTalon.getConfigurator().apply(limits);
}
```

- [ ] **Step 12: IntakePivot — add passthrough**

In `IntakePivot.java`, find the `io` field. Add before the closing `}`:

```java
public void setDefenseMode(boolean active) {
  io.setDefenseMode(active);
}
```

- [ ] **Step 13: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL.

- [ ] **Step 14: Run tests**

```bash
./gradlew test
```

Expected: all tests pass. The mock IO objects in existing tests implement the interface default no-op, so no test changes are needed.

- [ ] **Step 15: Commit**

```bash
git add \
  src/main/java/frc/robot/subsystems/shooter/ShooterIO.java \
  src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java \
  src/main/java/frc/robot/subsystems/shooter/Shooter.java \
  src/main/java/frc/robot/subsystems/hopper/HopperIO.java \
  src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java \
  src/main/java/frc/robot/subsystems/hopper/Hopper.java \
  src/main/java/frc/robot/subsystems/intake/IntakeRollerIO.java \
  src/main/java/frc/robot/subsystems/intake/IntakeRollerIOTalonFX.java \
  src/main/java/frc/robot/subsystems/intake/IntakeRoller.java \
  src/main/java/frc/robot/subsystems/intake/IntakePivotIO.java \
  src/main/java/frc/robot/subsystems/intake/IntakePivotIOTalonFX.java \
  src/main/java/frc/robot/subsystems/intake/IntakePivot.java
git commit -m "feat(subsystems): add defense mode current limit reconfiguration to shooter, hopper, intake"
```

---

## Task 6: Wire Defense Mode Toggle in RobotContainer

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`

- [ ] **Step 1: Add defenseModeActive field**

In `RobotContainer.java`, after the `private final CommandXboxController controller` field declaration, add:

```java
private boolean defenseModeActive = false;
```

- [ ] **Step 2: Wire Y button toggle**

In `configureButtonBindings()`, add at the end (before the closing `}`):

```java
// Y button: toggle defense mode (raises drive current, lowers other subsystem current)
controller
    .y()
    .onTrue(
        Commands.runOnce(
            () -> {
              defenseModeActive = !defenseModeActive;
              drive.setDefenseMode(defenseModeActive);
              shooter.setDefenseMode(defenseModeActive);
              hopper.setDefenseMode(defenseModeActive);
              intakeRoller.setDefenseMode(defenseModeActive);
              intakePivot.setDefenseMode(defenseModeActive);
            }));
```

- [ ] **Step 3: Reset defense mode on teleopInit**

In `teleopInit()`, after the existing `superstructure.setWantedSuperState(WantedState.INTAKING)` line, add:

```java
// Reset defense mode at the start of each teleop period
if (defenseModeActive) {
  defenseModeActive = false;
  drive.setDefenseMode(false);
  shooter.setDefenseMode(false);
  hopper.setDefenseMode(false);
  intakeRoller.setDefenseMode(false);
  intakePivot.setDefenseMode(false);
}
```

- [ ] **Step 4: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL.

- [ ] **Step 5: Run all tests**

```bash
./gradlew test
```

Expected: BUILD SUCCESSFUL, all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat(robot): wire Y button defense mode toggle, reset on teleopInit"
```

---

## Self-Review Notes

- Spec §1 (bypass flag): Task 1 ✓
- Spec §2 (MAX_MODULE_ANGULAR_VELOCITY 30→50): Task 1 ✓
- Spec §3 (Betaflight curve + LoggedTunableNumber): Task 2 uses plain `static final` constants since `LoggedTunableNumber` does not exist in this codebase. Values are in `DriveCommands.java` and require a redeploy to change. ✓
- Spec §4 (defense mode, all subsystems, Y toggle, teleopInit reset): Tasks 4-6 ✓
- Spec §5 (lock restructure + timing): Task 3 ✓
- `Module.periodic()` preserved as delegate → existing test call sites unaffected ✓
- `HopperIOTalonFX.topIndexerTalon` field is `public` (confirmed in source) → accessible in `setDefenseMode` ✓
- `IntakeRoller.io` field needs verification — check `IntakeRoller.java` for the field name before Task 5 Step 9. If the field is named differently, adjust accordingly.
