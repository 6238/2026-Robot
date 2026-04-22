# Swerve MotionMagicExpo + SysId Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Switch swerve steer control from PositionVoltage to MotionMagicExpoVoltage, fix the drive SysId log consumer, and add a steer SysId routine — all accessible as autonomous options.

**Architecture:** Four targeted file changes with no new files. MotionMagic config already exists in hardware — only the control request type changes. SysId routines follow the existing drive SysId pattern already in the codebase. No unit tests — all changes are hardware IO layer; verification is compilation + deploy.

**Tech Stack:** WPILib Command-based Java, CTRE Phoenix 6 TalonFX, AdvantageKit, WPILib SysId framework

---

### Task 1: Swap steer to MotionMagicExpoVoltage

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`

The MotionMagic config (Expo_kV, Expo_kA, CruiseVelocity, Acceleration) is already applied in the constructor at lines 147-151. Only the control request needs to change.

- [ ] **Step 1: Add the MotionMagicExpoVoltage import and field**

In `ModuleIOTalonFX.java`, add the import after the existing control request imports (around line 28):
```java
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
```

Add the field alongside the existing `positionVoltageRequest` field (around line 65):
```java
private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0.0);
```

- [ ] **Step 2: Swap the control request in setTurnPosition**

In `setTurnPosition` (around line 296), change the Voltage case from:
```java
case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
```
to:
```java
case Voltage -> motionMagicRequest.withPosition(rotation.getRotations());
```

The `TorqueCurrentFOC` case stays unchanged.

- [ ] **Step 3: Build to verify compilation**

```bash
./gradlew build -x test
```
Expected: `BUILD SUCCESSFUL`. If spotless fails first, run `./gradlew spotlessApply` then retry.

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java
git commit -m "feat(drive): switch steer to MotionMagicExpoVoltage"
```

---

### Task 2: Add Module getters and steer characterization method

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/Module.java`

These getters expose `inputs` fields needed by the Drive SysId log consumers. `runSteerCharacterization` is the counterpart to the existing `runCharacterization` (drive).

- [ ] **Step 1: Add getDriveAppliedVolts and getDrivePositionRad**

In `Module.java`, after the existing `getDriveVelocityRadPerSec()` method (around line 193), add:
```java
/** Returns the drive motor applied voltage. Used by SysId log consumer. */
public double getDriveAppliedVolts() {
  return inputs.driveAppliedVolts;
}

/** Returns the drive motor position in radians. Used by SysId log consumer. */
public double getDrivePositionRad() {
  return inputs.drivePositionRad;
}
```

- [ ] **Step 2: Add getTurnAppliedVolts**

After the new drive getters, add:
```java
/** Returns the turn motor applied voltage. Used by SysId log consumer. */
public double getTurnAppliedVolts() {
  return inputs.turnAppliedVolts;
}
```

- [ ] **Step 3: Add runSteerCharacterization**

After the existing `runCharacterization(double output)` method (around line 124), add:
```java
/** Runs the steer motor open-loop for SysId characterization. Stops drive explicitly. */
public void runSteerCharacterization(double output) {
  io.setDriveOpenLoop(0.0);
  io.setTurnOpenLoop(output);
}
```

- [ ] **Step 4: Build to verify compilation**

```bash
./gradlew build -x test
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Module.java
git commit -m "feat(drive): add SysId getters and steer characterization to Module"
```

---

### Task 3: Fix drive SysId log consumer

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

The existing `sysId` field passes `null` as its log consumer (line 250). WPILib SysId Analyzer needs voltage, position, and velocity logged per motor — replace null with an explicit consumer.

- [ ] **Step 1: Replace the null log consumer in the sysId field**

Find the `sysId` field initialization (around lines 242-250):
```java
sysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
```

Replace with:
```java
sysId =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)),
            (log) -> {
              for (int i = 0; i < 4; i++) {
                log.motor("drive-" + i)
                    .voltage(Volts.of(modules[i].getDriveAppliedVolts()))
                    .angularPosition(Radians.of(modules[i].getDrivePositionRad()))
                    .angularVelocity(RadiansPerSecond.of(modules[i].getDriveVelocityRadPerSec()));
              }
            },
            this));
```

`Volts`, `Radians`, `RadiansPerSecond` are all available via the existing `import static edu.wpi.first.units.Units.*` at the top of the file.

- [ ] **Step 2: Build to verify compilation**

```bash
./gradlew build -x test
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "fix(drive): provide SysId log consumer for drive motors"
```

---

### Task 4: Add steer SysId routine and command factories

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

Mirror the existing drive SysId pattern: a `SysIdRoutine` field, a `runSteerCharacterization` helper, and two public command factory methods.

- [ ] **Step 1: Add the sysIdSteer field**

After the existing `private final SysIdRoutine sysId;` field declaration (around line 141), add:
```java
private final SysIdRoutine sysIdSteer;
```

- [ ] **Step 2: Initialize sysIdSteer in the constructor**

After the `sysId = new SysIdRoutine(...)` block (around line 255), add:
```java
sysIdSteer =
    new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null,
            (state) -> Logger.recordOutput("Drive/SysIdSteerState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runSteerCharacterization(voltage.in(Volts)),
            (log) -> {
              for (int i = 0; i < 4; i++) {
                log.motor("steer-" + i)
                    .voltage(Volts.of(modules[i].getTurnAppliedVolts()))
                    .angularPosition(Radians.of(modules[i].getAngle().getRadians()))
                    .angularVelocity(RadiansPerSecond.of(modules[i].getSteerVelocityRadPerSec()));
              }
            },
            this));
```

Note: `getAngle().getRadians()` returns accumulated radians (not wrapped) because Phoenix 6 `turnPosition` tracks total rotation.

- [ ] **Step 3: Add runSteerCharacterization helper**

After the existing `runCharacterization(double output)` method in Drive.java, add:
```java
private void runSteerCharacterization(double output) {
  for (var module : modules) {
    module.runSteerCharacterization(output);
  }
}
```

- [ ] **Step 4: Add sysIdSteerQuasistatic and sysIdSteerDynamic command factories**

After the existing `sysIdDynamic` method (around line 438), add:
```java
/** Returns a command to run a steer quasistatic SysId test in the specified direction. */
public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
  return run(() -> runSteerCharacterization(0.0))
      .withTimeout(1.0)
      .andThen(sysIdSteer.quasistatic(direction));
}

/** Returns a command to run a steer dynamic SysId test in the specified direction. */
public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
  return run(() -> runSteerCharacterization(0.0))
      .withTimeout(1.0)
      .andThen(sysIdSteer.dynamic(direction));
}
```

- [ ] **Step 5: Build to verify compilation**

```bash
./gradlew build -x test
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "feat(drive): add steer SysId routine and command factories"
```

---

### Task 5: Register SysId autos in RobotContainer

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`

The drive SysId autos (lines 221-229) are already registered. Add 4 steer options immediately after them, inside the existing `try` block.

- [ ] **Step 1: Add 4 steer SysId auto options**

Find the existing drive SysId block (around lines 221-229):
```java
tempChooser.addOption(
    "Drive SysId (Quasistatic Forward)",
    drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
tempChooser.addOption(
    "Drive SysId (Quasistatic Reverse)",
    drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
tempChooser.addOption(
    "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
tempChooser.addOption(
    "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
```

Immediately after that block (before the `} catch` line), add:
```java
tempChooser.addOption(
    "Steer SysId (Quasistatic Forward)",
    drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
tempChooser.addOption(
    "Steer SysId (Quasistatic Reverse)",
    drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
tempChooser.addOption(
    "Steer SysId (Dynamic Forward)",
    drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
tempChooser.addOption(
    "Steer SysId (Dynamic Reverse)",
    drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));
```

- [ ] **Step 2: Build to verify compilation**

```bash
./gradlew build -x test
```
Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java
git commit -m "feat(drive): register steer SysId routines as auto options"
```

---

## Post-Deployment: Applying SysId Results

After running characterization on hardware, update both tuner constants files:

**Drive results** → `COMPTunerConstants.java` and `PRACTICETunerConstants.java`:
```java
private static final Slot0Configs driveGains =
    new Slot0Configs()
        .withKP(/* keep existing */)
        .withKS(/* SysId result */)
        .withKV(/* SysId result */)
        .withKA(/* SysId result */);
```

**Steer results** → same files, two places:
```java
private static final Slot0Configs steerGains =
    new Slot0Configs()
        .withKP(30).withKD(1.0)  // keep existing PD
        .withKS(/* SysId result */)
        .withKV(/* SysId result */)
        .withKA(/* SysId result */);
```
And in `ModuleIOTalonFX.java` constructor:
```java
turnConfig.MotionMagic.MotionMagicExpo_kV = /* SysId kV result × steerGearRatio */;
turnConfig.MotionMagic.MotionMagicExpo_kA = /* SysId kA result */;
```

**Unit conversion:** WPILib SysId Analyzer outputs kS (V), kV (V·s/rad), kA (V·s²/rad). Phoenix 6 `Slot0Configs` and `MotionMagicExpo_kV` use V/(rot/s) = V·s/rotation. Convert: multiply SysId kV and kA by 2π before entering into Phoenix configs. kS is volts and needs no conversion.
