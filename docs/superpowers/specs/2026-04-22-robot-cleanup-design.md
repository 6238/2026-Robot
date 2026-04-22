# Robot Cleanup & Performance Design
**Date:** 2026-04-22
**Branch:** Optimization

## Problem Summary

Multiple bugs and performance issues identified across Superstructure, Drive, and hardware IO layers. Primary pain points:

1. Manual/pit shoot completely broken — robot refuses to fire without hub alignment
2. Superstructure state machine has double-transition bug and duplicated logic
3. Left bumper intake toggle desyncs when interrupted by shooting commands
4. CAN bus over-subscribed — 10.9% of loop cycles exceed 20ms budget
5. PathPlanner NavMesh blocks main thread at startup (17.4s first loop spike)
6. Dead code (~150 lines) throughout ShotPlanner, GratuitousLighting, Superstructure
7. Battery sags to 7.24V during shooting — low-voltage alert missing

---

## Section 1: Superstructure State Machine Refactor

### Enum Changes

Remove `MANUAL_SHOOTING` from `WantedState`. Add `PIT_SHOOT`.

```
WantedState:  IDLE | INTAKING | SHOOTING | PASSING | SHOOT_INTAKE | PASS_INTAKE | PIT_SHOOT
CurrentState: IDLE | INTAKING | SPINNING_UP | SHOOTING | PASSING | PIT_SHOOTING
```

### Fix Double-Transition Bug

`setWantedSuperState()` currently calls `handleWantedState()` AND `periodic()` calls it again. Fix: `setWantedSuperState()` sets the field only. `periodic()` calls `handleWantedState()` once per loop, then `applyStates()`.

```java
public void setWantedSuperState(WantedState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
    // removed: handleWantedState()
}

@Override
public void periodic() {
    // update shotSetpoint for all active states...
    handleWantedState();
    applyStates();
    // logging...
}
```

### PIT_SHOOT Transition

In `handleWantedState()`, `PIT_SHOOT` transitions directly to `PIT_SHOOTING` — no SPINNING_UP wait.

```java
case PIT_SHOOT:
    if (currentSuperState != CurrentState.PIT_SHOOTING) {
        currentSuperState = CurrentState.PIT_SHOOTING;
        crawlUpScheduled = false;
        oscTimer = 0.0;
        noShotTimer.restart();
    }
    break;
```

### Fix shotSetpoint for PIT_SHOOT

In `periodic()`, add PIT_SHOOT to the setpoint update block:

```java
if (wantedSuperState == WantedState.PIT_SHOOT) {
    shotSetpoint = new ShotSetpoint(
        RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get()),
        RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get()),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        drive.getPose(),           // rotation target = current pose → hub error always 0
        drive.getChassisSpeeds(),
        drive.getPose().getTranslation()
    );
}
```

Setting `robotPose = drive.getPose()` means `checkHubTolerance()` error = 0° — hub condition trivially satisfied if ever called.

### Fix crawlUpScheduled Gate

Currently gates on `readyToShoot()` which requires both flywheel speed AND hub tolerance. Change to `shooter.flywheelUpToSpeed()` only in SHOOTING and PASSING cases — indexers start as soon as flywheel is ready regardless of heading:

```java
if (shooter.flywheelUpToSpeed() && !crawlUpScheduled) {
    crawlUpScheduled = true;
    // ...
}
```

Hub tolerance still gates the SPINNING_UP → SHOOTING transition. This is correct. It just no longer prevents the indexer from pre-staging.

### Extract Shared Feeding Logic

SHOOTING and PASSING `applyStates()` cases are ~80 lines of near-identical code. Extract:

```java
private void applyFeedingLogic(boolean intakeAlso) {
    if (intakeAlso) intakeRoller.spin();
    shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
    shooter.setFeederSpeed(shotSetpoint.feederSpeed);

    boolean tooClose = isTooCloseToHub();
    hopper.setTopIndexerSpeed(RotationsPerSecond.of(
        tooClose ? 0 : HopperConstants.TOP_INDEXER_SPEED.get()));
    hopper.setIndexerSpeed(RotationsPerSecond.of(
        tooClose ? 0 : HopperConstants.INDEXER_SPEED.get()));

    if (shooter.flywheelUpToSpeed() && !crawlUpScheduled) {
        crawlUpScheduled = true;
        oscTimer = 0.0;
        push254Phase = Push254Phase.PUSHING;
        push254JamDebouncer.calculate(false);
    }

    // beam-break rising-edge / intakeWaitingForNextBall logic
    boolean beamBreak = shooter.isShooting();
    boolean risingEdge = beamBreak && !prevBeamBreak;
    if (risingEdge) {
        intakeWaitingForNextBall = !intakeWaitingForNextBall;
        if (intakeWaitingForNextBall) {
            intakeDownTimer.restart();
            push254Phase = Push254Phase.PUSHING;
            push254JamDebouncer.calculate(false);
        }
        noShotTimer.restart();
    }
    prevBeamBreak = beamBreak;

    if (intakeWaitingForNextBall && intakeDownTimer.hasElapsed(0.25)) {
        intakeWaitingForNextBall = false;
        push254Phase = Push254Phase.PUSHING;
        push254JamDebouncer.calculate(false);
    }

    // Suppress oscillation when actively intaking (intake pivot is in use)
    boolean suppressOscillation =
        wantedSuperState == WantedState.SHOOT_INTAKE ||
        wantedSuperState == WantedState.PASS_INTAKE;

    if (crawlUpScheduled && !suppressOscillation) {
        if (intakeWaitingForNextBall) {
            intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
        } else {
            if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.PUSH_254) {
                apply254Push();
            } else {
                applyPivotOscillate();
            }
        }
    }

    simulateShot();
}
```

SHOOTING case calls `applyFeedingLogic(wantedSuperState == WantedState.SHOOT_INTAKE)`.
PASSING case calls `applyFeedingLogic(true)` — always spins intake roller; oscillation still suppressed when `PASS_INTAKE`.
PIT_SHOOTING does NOT call `applyFeedingLogic` — has its own simple case (see below).

SHOOTING case retains the `if (!checkHubTolerance()) { currentSuperState = SPINNING_UP; }` guard. PIT_SHOOTING does NOT have this guard.

### PIT_SHOOTING applyStates Case

Simple case — no hub guard, no `isTooCloseToHub()`, no beam-break logic:

```java
case PIT_SHOOTING:
    shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
    if (shooter.flywheelUpToSpeed()) {
        hopper.setIndexerSpeed(RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get()));
        hopper.setTopIndexerSpeed(RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get()));
        shooter.setFeederSpeed(shotSetpoint.feederSpeed);
    }
    break;
```

Flywheel must reach speed before indexers/feeder engage — only safety gate. No hub tolerance, no proximity check.

### Make Subsystem Fields Private

`drive`, `shooter`, `hopper`, `intake`, `intakeRoller` fields changed from `public` to `private`. `currentSuperState` stays `public` — needed by `GratuitousLighting` and sim.

---

## Section 2: Button Binding Fixes

### Left Bumper Intake — Fix Toggle Desync

Current `toggleOnTrue(wantIntaking())` desyncs when shooting commands interrupt `wantIntaking()` startEnd, firing the end lambda (`setIdle`) out of turn.

Replace with explicit pair:

```java
controller.leftBumper()
    .whileTrue(superstructure.setWantedSuperStateCommand(() -> WantedState.INTAKING))
    .onFalse(superstructure.setWantedSuperStateCommand(() -> WantedState.IDLE));
```

`setWantedSuperStateCommand` is `runOnce` — completes immediately, holds no subsystem requirement, cannot be interrupted. No toggle state to desync.

### Remove .until(leftBumper) from Shoot Bindings

The `.until(() -> controller.leftBumper().getAsBoolean())` on right trigger/bumper parallel commands was compensating for toggle desync. No longer needed — remove it.

### D-Pad Left — Bind to PIT_SHOOT

```java
controller.povLeft()
    .whileTrue(superstructure.setWantedSuperStateCommand(() -> WantedState.PIT_SHOOT))
    .onFalse(superstructure.setWantedSuperStateCommand(() -> WantedState.IDLE));
```

---

## Section 3: Performance & Hardware Fixes

### Drive — Eliminate Per-Call Array Allocations

`getModuleStates()` and `getModulePositions()` allocate new arrays every call. `getChassisSpeeds()` calls `getModuleStates()` every loop.

Pre-allocate as fields, mutate in place:

```java
private final SwerveModuleState[] moduleStatesCache = new SwerveModuleState[4];
private final SwerveModulePosition[] modulePositionsCache = new SwerveModulePosition[4];

// constructor: populate with initial instances
for (int i = 0; i < 4; i++) {
    moduleStatesCache[i] = new SwerveModuleState();
    modulePositionsCache[i] = new SwerveModulePosition();
}

private SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < 4; i++) moduleStatesCache[i] = modules[i].getState();
    return moduleStatesCache;
}
```

### Hub Rotation Tolerances — Fix Dead Dynamic Tolerance

Both tolerances currently 3°. `getDynamicHubToleranceDegrees()` returns same value regardless of conditions.

```java
// ShooterConstants.java
public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(5);       // far + slow
public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(3); // near hub or fast
```

Now near-hub (<3m) or high-speed (>2.5 m/s) → 3°, otherwise → 5°. Gives more forgiveness at range.

### Defense Mode — Async CAN Calls

`setDefenseMode()` in `HopperIOTalonFX` and `IntakePivotIOTalonFX` uses blocking `.apply()`. Change to zero-timeout async:

```java
intakeArmTalon.getConfigurator().apply(limits, 0.0);
```

Called once on Y button press — async is fine here.

### Hopper Dead Zone — Fix ±2 RPS Stop Threshold

`isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(2))` stops motor for any command ≤ 2 RPS. Eliminates low-speed indexing.

```java
// Change in HopperIOTalonFX.setIndexerSpeed() and setTopIndexerSpeed()
if (speed.isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(0.5))) {
    indexerTalon.stopMotor();
}
```

### Low Battery Alert

In `RobotContainer`, add:

```java
private final Alert batteryLowAlert =
    new Alert("Battery voltage low — check before match", AlertType.kWarning);
```

Set in `periodic()`:
```java
batteryLowAlert.set(RobotController.getBatteryVoltage() < 10.5);
```

### PathPlanner Startup Spike Fix

First robot loop blocked 17.4 seconds loading NavMesh. Pre-warm during robotInit:

```java
// In Robot.java robotInit():
PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();
```

Runs NavMesh initialization off the first enabled loop, eliminating the spike.

---

## Section 4: Dead Code Removal

### ShotPlanner — Remove Polynomial Regression (~90 lines)

Remove entirely:
- `createPolynomialShotSetpoint()`
- `W_SPEED`, `W_HOOD`, `W_HEADING` weight arrays
- `buildFeatures()`
- `dotProduct()`
- `POLY_DIM` constant
- `flywheelMPStoRPS()` — only called by polynomial path

Never called anywhere. Replaced by `createShotSetpoint()` lookup-table approach.

### GratuitousLighting — Remove Dead Field

```java
// Remove:
private double shootingAnimStart = -1;  // assigned, never read
```

### Superstructure — Remove Commented-Out Blocks

Remove all multi-line commented-out code blocks:
- Top-indexer jam recovery block (~15 lines)
- Feeder velocity compensation block (~12 lines)
- Hopper spinup in IDLE (~3 lines)
- Commented-out D-pad bindings in RobotContainer (~9 lines)

These are not stubs — they were abandoned experiments. Use git history if ever needed.

### BEAM_BREAK_DIO_PORT TODO

Remove TODO comment. Either confirm port 0 is correct or set the actual port.

### Logging — Remove Side Effects from Predicates

`readyToShoot()` and `isPrettyMuchCloseToTargetButNotQuite()` both call `Logger.recordOutput()` for the same fields. Move all logging to `periodic()`. Both methods become pure boolean functions.

---

## Section 5: Logging & Observability

### Pit Shoot Dashboard Feedback

In `periodic()`, add when in `PIT_SHOOTING`:

```java
Logger.recordOutput("Superstructure/PitShootActive",
    currentSuperState == CurrentState.PIT_SHOOTING);
Logger.recordOutput("Superstructure/FlywheelReadyForPitShoot",
    shooter.flywheelUpToSpeed());
```

Drive team sees clear ready indicator on dashboard during pit testing.

### Drive Timing — Remove Characterization Scaffolding

Remove `double a`, `b`, `c` timing checkpoints and `Drive/RefreshTimeMs`, `Drive/OdometryLockMs`, `Drive/InputsMs` outputs. Keep `Drive/PeriodicTimeMs` only.

### CAN Bus Utilization Alert

```java
private final Alert canHighAlert =
    new Alert("CAN bus utilization high (>80%)", AlertType.kWarning);

// In periodic():
canHighAlert.set(
    new CANBus(ShooterConstants.CAN_BUS.getName()).getStatus().BusUtilization > 0.8);
```

---

## Section 6: CAN Signal Rate Tuning

### Problem

All IO implementations call `setUpdateFrequencyForAll(50)` — 50Hz for every signal regardless of need. During heavy shooting + driving, canivore hits 100% utilization, causing `BaseStatusSignal.refreshAll()` to block and causing 10.9% of cycles to exceed the 20ms budget.

### Signal Tiers

| Tier | Rate | Rationale |
|------|------|-----------|
| Odometry | 250Hz | Drive position only, via PhoenixOdometryThread (unchanged) |
| Control | 50Hz | Signals used directly for control decisions each loop |
| Monitoring | 20Hz | Signals read for logging/alerts only |

### Changes Per IO

**`ShooterIOTalonFX`**
- 50Hz: `flywheelVelocity`, `feederVelocity`
- 20Hz: `flywheelAppliedVoltage`, `flywheelSupplyCurrent`, `flywheel2SupplyCurrent`, `feederAppliedVoltage`, `feederSupplyCurrent`
- Note: beam break is roboRIO DIO — not a CAN signal, untouched

**`HopperIOTalonFX`**
- 50Hz: `indexerVelocity`, `topIndexerVelocity`
- 20Hz: `indexerVoltage`, `indexerSupplyCurrent`, `topIndexerVoltage`, `topIndexerSupplyCurrent`

**`IntakePivotIOTalonFX`**
- 50Hz: `intakeArmPosition`, `intakeArmVelocity`
- 20Hz: `intakeArmVoltage`, `intakeArmSupplyCurrent`, `intakeArmStatorCurrent`

**`IntakeRollerIOTalonFX`**
- 20Hz: all signals (velocity, current, voltage — roller not used for tight control loop)

**`ModuleIOTalonFX`**
- 250Hz: drive position (odometry thread, unchanged)
- 50Hz: drive velocity, steer position, steer velocity
- 20Hz: drive applied voltage, drive supply current, steer applied voltage, steer supply current

### Implementation Pattern

Replace `statusSignalCollector.setUpdateFrequencyForAll(50)` with explicit calls:

```java
// Control signals
BaseStatusSignal.setUpdateFrequencyForAll(50,
    flywheelVelocity, feederVelocity);

// Monitoring signals
BaseStatusSignal.setUpdateFrequencyForAll(20,
    flywheelAppliedVoltage, flywheelSupplyCurrent,
    feederAppliedVoltage, feederSupplyCurrent);
```

Removes ~12 signals from the 50Hz CAN budget per loop cycle. Frees ~600 CAN frames/sec on canivore. Expected to eliminate the 100% utilization windows.

---

## Files Changed

| File | Change |
|------|--------|
| `Superstructure.java` | State machine refactor, PIT_SHOOT, fix double-call, extract applyFeedingLogic, private fields |
| `RobotContainer.java` | Fix button bindings, add alerts, PathPlanner warmup, remove dead bindings |
| `ShotPlanner.java` | Remove polynomial dead code |
| `ShooterConstants.java` | Fix HUB_ROTATION_TOLERANCE values, remove BEAM_BREAK TODO |
| `Drive.java` | Pre-allocate module state/position arrays, remove timing scaffolding |
| `HopperIOTalonFX.java` | Async defense mode, fix dead zone, signal rate tuning |
| `IntakePivotIOTalonFX.java` | Async defense mode, signal rate tuning |
| `ShooterIOTalonFX.java` | Signal rate tuning |
| `IntakeRollerIOTalonFX.java` | Signal rate tuning |
| `ModuleIOTalonFX.java` | Signal rate tuning |
| `GratuitousLighting.java` | Remove dead field |
| `Robot.java` | PathPlanner warmup command |

## Out of Scope

- Battery hardware — 7.24V sag is a weak battery, not a code issue
- Vision tuning — std dev / filter parameters unchanged
- Auto path changes
- Shooter map tuning
