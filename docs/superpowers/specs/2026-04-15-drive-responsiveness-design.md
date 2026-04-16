# Drive Responsiveness Improvements

**Date:** 2026-04-15
**Branch:** TestModeVendorDep
**Problem:** After playing defense, drivebase feels floaty and unresponsive. Root causes: loop overruns spiking to ~19ms, setpoint generator lag (~100ms before setpoints change), suboptimal stick curve, and no defense-mode current profile.

---

## 1. Setpoint Generator Bypass Flag

**File:** `DriveConstants.java`, `Drive.java`

Add a tunable boolean to `DriveConstants`:

```java
public static final LoggedTunableNumber BYPASS_SETPOINT_GENERATOR =
    new LoggedTunableNumber("Drive/BypassSetpointGenerator", 0);
```

In `Drive.runVelocity(ChassisSpeeds, DriveFeedforwards)`, gate the setpoint generator:

```java
if (DriveConstants.BYPASS_SETPOINT_GENERATOR.get() > 0.5) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, getMaxLinearSpeedMetersPerSec());
    for (int i = 0; i < 4; i++) modules[i].runSetpoint(new SwerveModuleState(...), 0.0);
} else {
    // existing setpoint generator path
}
```

Log the bypass state each cycle so it's visible in logs. PathPlanner auto still uses the non-bypass path (it passes feedforwards which distinguish the call).

---

## 2. Setpoint Generator Max Steer Velocity

**File:** `DriveConstants.java`

Raise `MAX_MODULE_ANGULAR_VELOCITY` from `30` → `50` rad/s.

At 30 rad/s, a 90° reorientation takes ~52ms. Combined with cosine scaling suppressing drive output while modules aren't pointed correctly, this compounds to ~100ms of perceived lag. At 50 rad/s, the same reorientation takes ~31ms.

```java
public static final AngularVelocity MAX_MODULE_ANGULAR_VELOCITY = RadiansPerSecond.of(50);
```

---

## 3. Stick Curves — Betaflight Actual Rate Model

**File:** `DriveCommands.java`

Replace the quadratic curve in `getLinearVelocityFromJoysticks` and the omega squaring with the Betaflight Actual rate model:

```
output(x) = (centerSens * x + (maxRate - centerSens) * pow(x, expo)) / maxRate
```

Where `x` ∈ [0, 1] (post-deadband stick magnitude), output ∈ [0, 1].

Add three `LoggedTunableNumber` constants in `DriveCommands`:

```java
private static final LoggedTunableNumber LINEAR_CENTER_SENS =
    new LoggedTunableNumber("Drive/Curve/LinearCenterSens", 0.3);
private static final LoggedTunableNumber LINEAR_MAX_RATE =
    new LoggedTunableNumber("Drive/Curve/LinearMaxRate", 1.0);
private static final LoggedTunableNumber LINEAR_EXPO =
    new LoggedTunableNumber("Drive/Curve/LinearExpo", 3.0);
```

The same parameters are reused for omega (angular). Starting values give ~30% speed at 50% stick (more precise center) while still reaching 100% at full deflection.

Update `getLinearVelocityFromJoysticks` to apply the curve to `linearMagnitude` after deadband. Update the omega shaping in `joystickDrive` and `joystickDriveRobotRelative`.

---

## 4. Defense Mode

### 4a. Drive subsystem

**File:** `Drive.java`, `ModuleIOTalonFX.java`

`ModuleIOTalonFX` exposes a `setDriveCurrentLimits(double supplyAmps, double statorAmps)` method that calls `driveTalon.getConfigurator().apply(new CurrentLimitsConfigs()...)` at runtime. This does not block (Phoenix 6 configurator apply is async CAN write).

`Drive` adds:

```java
private boolean defenseModeActive = false;

public void setDefenseMode(boolean active) {
    if (active == defenseModeActive) return;
    defenseModeActive = active;
    for (var module : modules) {
        module.setDriveCurrentLimits(
            active ? 80.0 : 55.0,   // supply
            active ? 120.0 : 102.0  // stator
        );
    }
    Logger.recordOutput("Drive/DefenseMode", defenseModeActive);
}
```

Add `setDriveCurrentLimits` to the `ModuleIO` interface with a default no-op so simulation and tests are unaffected.

### 4b. Other subsystems

Add `default void setDefenseMode(boolean active) {}` to `ShooterIO`, `HopperIO`, `IntakeRollerIO`, `IntakePivotIO`. TalonFX implementations apply reduced current limits (specific values to be determined at tuning time; suggested starting point: reduce supply by ~30% from normal).

Each subsystem class (`Shooter`, `Hopper`, `IntakeRoller`, `IntakePivot`) gets a `setDefenseMode(boolean)` passthrough to its IO.

### 4c. RobotContainer wiring

Y button toggles defense mode on all subsystems:

```java
// Field in RobotContainer
private boolean defenseModeActive = false;

// In configureButtonBindings():
controller.y().onTrue(Commands.runOnce(() -> {
    defenseModeActive = !defenseModeActive;
    drive.setDefenseMode(defenseModeActive);
    shooter.setDefenseMode(defenseModeActive);
    hopper.setDefenseMode(defenseModeActive);
    intakeRoller.setDefenseMode(defenseModeActive);
    intakePivot.setDefenseMode(defenseModeActive);
}));
```

On `teleopInit()`, reset defense mode to false (already called in `RobotContainer.teleopInit()`).

---

## 5. Loop Overrun Reduction

**Files:** `Drive.java`, `Module.java`

### Root cause

The `odometryLock` is held in `Drive.periodic()` during both:
1. `io.updateInputs(inputs)` — reads cached CAN signal values into the inputs struct (fast)
2. `Logger.processInputs(...)` — AdvantageKit serialization of all input fields (variable; can spike)

The `PhoenixOdometryThread` (running at 250Hz, every 4ms) also acquires `odometryLock` to drain the signal queues. When the main thread holds the lock for Logger serialization, the odometry thread blocks and vice versa, creating contention spikes.

### Fix: split Module.periodic() into hardware read + log

Add a `logInputs()` method to `Module` (call order: `updateHardwareInputs()` under lock, then `logInputs()` after lock release):

```java
// Module.java — new method
public void updateHardwareInputs() {
    io.updateInputs(inputs);
}

// Module.java — rename/refactor existing periodic():
public void logAndProcessInputs() {
    Logger.processInputs(logKey, inputs);
    // odometry position calc + alert updates (no hardware I/O)
}
```

`Module.periodic()` becomes a delegate that calls both (preserving the existing call site in tests):

```java
public void periodic() {
    updateHardwareInputs();
    logAndProcessInputs();
}
```

Restructure `Drive.periodic()`:

```java
// Under lock: hardware reads only
odometryLock.lock();
gyroIO.updateInputs(gyroInputs);
for (var module : modules) module.updateHardwareInputs();
odometryLock.unlock();

// Outside lock: logging + odometry math (no hardware I/O needed)
Logger.processInputs("Drive/Gyro", gyroInputs);
for (var module : modules) module.logAndProcessInputs();
```

This cuts the lock hold time by ~50%, reducing the window where the main thread and odometry thread contend.

### Profiling

Add cycle-time profiling around the main sections to confirm in logs:

```java
Logger.recordOutput("Drive/LoopMs/TotalMs", (Timer.getFPGATimestamp() - loopStart) * 1000);
```

Measure: lock acquisition time, hardware read time, Logger serialization time. Remove profiling before competition if log bandwidth is a concern (check `Constants.MINIMAL_LOGGING`).

---

## Out of scope

- Module PID tuning (steer KP/KV) — handled separately by driver/tuner
- PathPlanner auto behavior — setpoint generator bypass does not affect auto path
