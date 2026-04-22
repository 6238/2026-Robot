# Swerve MotionMagicExpo + SysId Design

**Date:** 2026-04-18
**Scope:** Switch steer control to MotionMagicExpoVoltage; wire SysId routines for drive and steer motors accessible as auto options.

---

## 1. MotionMagicExpoVoltage Swap

**File:** `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`

The MotionMagic config is already applied (lines 147-151) with placeholder values:
- `Expo_kV = 0.12 × steerGearRatio = 1.452`
- `Expo_kA = 0.1`

These placeholders stay until SysId produces real values.

**Changes:**
- Import `com.ctre.phoenix6.controls.MotionMagicExpoVoltage`
- Add field: `private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0.0)`
- In `setTurnPosition`, Voltage case: replace `positionVoltageRequest.withPosition(...)` with `motionMagicRequest.withPosition(...)`
- `PositionTorqueCurrentFOC` path is unchanged

---

## 2. Drive SysId — Fix Log Consumer

**File:** `src/main/java/frc/robot/subsystems/drive/Module.java`

Add getters needed by the log consumer:
- `getDriveAppliedVolts()` → `inputs.driveAppliedVolts`
- `getDrivePositionRad()` → `inputs.drivePositionRad`

(`getDriveVelocityRadPerSec()` already exists.)

**File:** `src/main/java/frc/robot/subsystems/drive/Drive.java`

Replace the existing `null` log consumer in `sysId` with:
```java
(log) -> {
    for (int i = 0; i < 4; i++) {
        log.motor("drive-" + i)
            .voltage(Volts.of(modules[i].getDriveAppliedVolts()))
            .angularPosition(Radians.of(modules[i].getDrivePositionRad()))
            .angularVelocity(RadiansPerSecond.of(modules[i].getDriveVelocityRadPerSec()));
    }
}
```

---

## 3. Steer SysId — New Routine

**File:** `src/main/java/frc/robot/subsystems/drive/Module.java`

- Add `getTurnAppliedVolts()` → `inputs.turnAppliedVolts`
- Add `runSteerCharacterization(double output)`:
  - Calls `io.setDriveOpenLoop(0.0)` — stops drive explicitly
  - Calls `io.setTurnOpenLoop(output)` — runs steer open-loop

**File:** `src/main/java/frc/robot/subsystems/drive/Drive.java`

- Add field `sysIdSteer` (`SysIdRoutine`) with log consumer logging all 4 steer motors:
  - voltage: `getTurnAppliedVolts()`
  - angular position: `getAngle().getRadians()` (accumulated, not wrapped — Phoenix 6 `turnPosition` is the accumulated rotor position divided by gear ratio)
  - angular velocity: `getSteerVelocityRadPerSec()`
- Add `runSteerCharacterization(double output)` — calls `module.runSteerCharacterization(output)` for all 4 modules
- Add command factories: `sysIdSteerQuasistatic(Direction)` and `sysIdSteerDynamic(Direction)`, mirroring the existing drive pattern — 1-second open-loop-zero warmup then the SysId test

---

## 4. RobotContainer — Auto Options

**File:** `src/main/java/frc/robot/RobotContainer.java`

Register 8 SysId test phases into the existing PathPlanner `autoChooser`:

| Auto Name | Command |
|---|---|
| `Drive SysId (Quasistatic Forward)` | `drive.sysIdQuasistatic(kForward)` |
| `Drive SysId (Quasistatic Backward)` | `drive.sysIdQuasistatic(kReverse)` |
| `Drive SysId (Dynamic Forward)` | `drive.sysIdDynamic(kForward)` |
| `Drive SysId (Dynamic Backward)` | `drive.sysIdDynamic(kReverse)` |
| `Steer SysId (Quasistatic Forward)` | `drive.sysIdSteerQuasistatic(kForward)` |
| `Steer SysId (Quasistatic Backward)` | `drive.sysIdSteerQuasistatic(kReverse)` |
| `Steer SysId (Dynamic Forward)` | `drive.sysIdSteerDynamic(kForward)` |
| `Steer SysId (Dynamic Backward)` | `drive.sysIdSteerDynamic(kReverse)` |

---

## 5. Running SysId — Procedure

### Drive characterization
- Robot **on the ground**, clear straight path (~5m minimum)
- Run all 4 phases in order: quasistatic forward → backward → dynamic forward → backward
- Each phase: select auto, enable, let it run to completion or disable when robot reaches end of space
- Export `.wpilog` → WPILib SysId Analyzer → select all 4 `drive-*` motors → fit → get kS, kV, kA

### Steer characterization
- Robot **on ground or in air** (steer load is not weight-dependent)
- Same 4-phase sequence
- Export `.wpilog` → SysId Analyzer → select all 4 `steer-*` motors → fit → get kS, kV, kA

### Applying results
After characterization, update `COMPTunerConstants.java` (and `PRACTICETunerConstants.java`):
- Drive: `Slot0Configs` `kS`, `kV`, `kA`
- Steer: `Slot0Configs` `kS`, `kV`, `kA` + `MotionMagic.MotionMagicExpo_kV`, `MotionMagicExpo_kA`

---

## Files Changed

| File | Change |
|---|---|
| `ModuleIOTalonFX.java` | Add `MotionMagicExpoVoltage` request, swap steer control request |
| `Module.java` | Add 3 getters + `runSteerCharacterization` method |
| `Drive.java` | Fix drive SysId log consumer, add steer SysId routine + commands |
| `RobotContainer.java` | Register 8 SysId auto options |
| `COMPTunerConstants.java` | (post-SysId) Update kS/kV/kA for drive and steer, update Expo params |
