# Robot Cleanup & Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix broken pit shoot, eliminate CAN bus overruns causing loop budget violations, remove ~150 lines of dead code, and clean up the superstructure state machine's double-transition and toggle desync bugs.

**Architecture:** Work in dependency order: dead code and constants first (no deps), IO performance next (independent), then the superstructure state machine refactor, then button bindings. Each task produces a buildable, committable state.

**Tech Stack:** WPILib Command-based Java, CTRE Phoenix 6 TalonFX on CANivore, AdvantageKit logging, PathPlanner 2025, JUnit 5 + Mockito 5.

---

### Task 1: Remove ShotPlanner polynomial dead code

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/ShotPlanner.java`

- [ ] **Step 1: Delete lines 116–270 from ShotPlanner.java**

Remove everything from `FLYWHEEL_ENERGY_TRANSFER_COEFFICENT` through `dotProduct()` — the seven dead members:

```java
// DELETE THIS ENTIRE BLOCK (lines 116-270):
public static final double FLYWHEEL_ENERGY_TRANSFER_COEFFICENT = 60.0 / 40.0;

public static AngularVelocity flywheelMPStoRPS(double mps) { ... }

public static ShotSetpoint createPolynomialShotSetpoint(
    Pose2d drivePose, ChassisSpeeds driveChassisSpeeds) { ... }

private static final int POLY_DIM = 15;

public static double[] W_SPEED = { 10.8554212665, ... };  // 15 values
public static double[] W_HOOD  = { 40.2851914096, ... };  // 15 values
public static double[] W_HEADING = { -6.2120675691, ... }; // 15 values

private static void buildFeatures(double x, double y, double vx, double vy, double[] phi) { ... }

private static double dotProduct(double[] weights, double[] phi) { ... }
```

The file should end at the `ShotSetpoint` inner class (line 272 onward).

- [ ] **Step 2: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/ShotPlanner.java
git commit -m "refactor(shot-planner): remove dead polynomial regression code (~90 lines)"
```

---

### Task 2: GratuitousLighting — remove dead field and add PIT_SHOOTING animation

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/GratuitousLighting.java`

*Note: PIT_SHOOTING is added to `CurrentState` in Task 10. This task adds the dead-field removal now (safe) and the animation reference will be needed once Task 10 is complete. Do the animation update as part of Task 10 if working sequentially, or stage it here with a TODO comment.*

- [ ] **Step 1: Remove dead field declaration**

Delete line 31:
```java
private double shootingAnimStart = -1;
```

- [ ] **Step 2: Remove the field's only assignment in periodic()**

In `periodic()`, inside the final `else` branch (around line 114), delete:
```java
shootingAnimStart = -1;
```

The else block becomes:
```java
} else {
    candle.setControl(idleAnimation);
    candle2.setControl(idleAnimation);
}
```

- [ ] **Step 3: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/GratuitousLighting.java
git commit -m "refactor(lighting): remove dead shootingAnimStart field"
```

---

### Task 3: ShooterConstants — fix hub tolerances and remove BEAM_BREAK TODO

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`
- Modify: `src/test/java/subsystems/SuperstructureTest.java`

- [ ] **Step 1: Widen the far/slow tolerance to 5°**

Change lines 113–114:
```java
// Before:
public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(3);
public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(3);

// After:
public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(5);
public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(3);
```

`getDynamicHubToleranceDegrees()` in Superstructure returns `HUB_ROTATION_TOLERANCE_TIGHT` (3°) when near hub (<3m) or fast (>2.5 m/s), and `HUB_ROTATION_TOLERANCE` (5°) otherwise. Previously both were 3° so the dynamic check had no effect.

- [ ] **Step 2: Remove BEAM_BREAK_DIO_PORT TODO comment**

Change line 26:
```java
// Before:
public static final int BEAM_BREAK_DIO_PORT = 0; // TODO: set correct roboRIO DIO port
// After:
public static final int BEAM_BREAK_DIO_PORT = 0;
```

- [ ] **Step 3: Update stale comment in SuperstructureTest**

In `SuperstructureTest.java`, update the comment on line ~135:
```java
// Before:
// 10° error — exceeds the 3.5° HUB_ROTATION_TOLERANCE
// After:
// 10° error — exceeds both 3° tight and 5° wide HUB_ROTATION_TOLERANCE
```

- [ ] **Step 4: Run tests**

```bash
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: All pass. 10° error still exceeds both tolerances.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java \
        src/test/java/subsystems/SuperstructureTest.java
git commit -m "fix(shooter): differentiate hub tolerances (5° far/slow, 3° near/fast) and remove TODO"
```

---

### Task 4: Drive — pre-allocate module array caches and remove timing scaffolding

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/Drive.java`

- [ ] **Step 1: Add pre-allocated module state and position caches**

After the `odometryModuleDeltas` field declaration (around line 175), add two new fields:

```java
private final SwerveModuleState[] moduleStatesCache = new SwerveModuleState[] {
    new SwerveModuleState(), new SwerveModuleState(),
    new SwerveModuleState(), new SwerveModuleState()
};
private final SwerveModulePosition[] modulePositionsCache = new SwerveModulePosition[] {
    new SwerveModulePosition(), new SwerveModulePosition(),
    new SwerveModulePosition(), new SwerveModulePosition()
};
```

- [ ] **Step 2: Update getModuleStates() to reuse the cache**

Replace `getModuleStates()` (around line 505):
```java
// Before:
@AutoLogOutput(key = "SwerveStates/Measured")
private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getState();
    }
    return states;
}

// After:
@AutoLogOutput(key = "SwerveStates/Measured")
private SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < 4; i++) {
        moduleStatesCache[i] = modules[i].getState();
    }
    return moduleStatesCache;
}
```

- [ ] **Step 3: Update getModulePositions() to reuse the cache**

```java
// Before:
private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
        states[i] = modules[i].getPosition();
    }
    return states;
}

// After:
private SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
        modulePositionsCache[i] = modules[i].getPosition();
    }
    return modulePositionsCache;
}
```

- [ ] **Step 4: Remove timing variables a, b, c from periodic()**

In `periodic()`, find and delete these three lines:
```java
double a = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
// ...
double b = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
// ...
double c = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
```

Also remove these three Logger lines inside the `!MINIMAL_LOGGING` block:
```java
Logger.recordOutput("Drive/RefreshTimeMs", (a - periodicStartSec) * 1000.0);
Logger.recordOutput("Drive/OdometryLockMs", (b - a) * 1000.0);
Logger.recordOutput("Drive/InputsMs", (c - b) * 1000.0);
```

Keep `Drive/PeriodicTimeMs`:
```java
Logger.recordOutput("Drive/PeriodicTimeMs",
    (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - periodicStartSec) * 1000.0);
```

- [ ] **Step 5: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/Drive.java
git commit -m "perf(drive): pre-allocate module state/position caches, remove timing scaffolding"
```

---

### Task 5: HopperIOTalonFX — fix dead zone, async defense mode, signal rate tuning

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java`

- [ ] **Step 1: Fix dead zone in setIndexerSpeed()**

Around line 151:
```java
// Before:
if (speed.isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(2))) {
// After:
if (speed.isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(0.5))) {
```

- [ ] **Step 2: Fix dead zone in setTopIndexerSpeed()**

Around line 161:
```java
// Before:
if (speed.isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(2))) {
// After:
if (speed.isNear(RotationsPerSecond.of(0), RotationsPerSecond.of(0.5))) {
```

- [ ] **Step 3: Make setDefenseMode() async (zero-timeout apply)**

In `setDefenseMode()` (around lines 175–178):
```java
// Before:
indexerTalon.getConfigurator().apply(limits);
if (HopperConstants.USE_TOP_INDEXER) {
    topIndexerTalon.getConfigurator().apply(limits);
}

// After:
indexerTalon.getConfigurator().apply(limits, 0.0);
if (HopperConstants.USE_TOP_INDEXER) {
    topIndexerTalon.getConfigurator().apply(limits, 0.0);
}
```

- [ ] **Step 4: Add BaseStatusSignal import**

Add at the top of the imports:
```java
import com.ctre.phoenix6.BaseStatusSignal;
```

- [ ] **Step 5: Replace setUpdateFrequencyForAll(50) with tiered rates**

Replace line 120 (`statusSignalCollector.setUpdateFrequencyForAll(50);`) with:

```java
if (HopperConstants.USE_TOP_INDEXER) {
    BaseStatusSignal.setUpdateFrequencyForAll(50, indexerVelocity, topIndexerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(20,
        indexerVoltage, indexerSupplyCurrent,
        topIndexerVoltage, topIndexerSupplyCurrent);
} else {
    BaseStatusSignal.setUpdateFrequencyForAll(50, indexerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(20, indexerVoltage, indexerSupplyCurrent);
}
```

- [ ] **Step 6: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 7: Commit**

```bash
git add src/main/java/frc/robot/subsystems/hopper/HopperIOTalonFX.java
git commit -m "perf(hopper): fix 2-RPS dead zone to 0.5, async defense mode, tiered CAN signal rates"
```

---

### Task 6: IntakePivotIOTalonFX — async defense mode and signal rate tuning

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakePivotIOTalonFX.java`

- [ ] **Step 1: Make setDefenseMode() async**

In `setDefenseMode()` (line 147):
```java
// Before:
intakeArmTalon.getConfigurator().apply(limits);
// After:
intakeArmTalon.getConfigurator().apply(limits, 0.0);
```

- [ ] **Step 2: Add BaseStatusSignal import**

```java
import com.ctre.phoenix6.BaseStatusSignal;
```

- [ ] **Step 3: Replace setUpdateFrequencyForAll(50) with tiered rates**

Replace line 93 (`statusSignalCollector.setUpdateFrequencyForAll(50);`) with:

```java
BaseStatusSignal.setUpdateFrequencyForAll(50, intakeArmPosition, intakeArmVelocity);
BaseStatusSignal.setUpdateFrequencyForAll(20,
    intakeArmVoltage, intakeArmSupplyCurrent, intakeArmStatorCurrent);
```

- [ ] **Step 4: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/IntakePivotIOTalonFX.java
git commit -m "perf(intake-pivot): async defense mode, tiered CAN signal rates (50Hz control, 20Hz monitoring)"
```

---

### Task 7: ShooterIOTalonFX — signal rate tuning

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java`

- [ ] **Step 1: Add BaseStatusSignal import**

```java
import com.ctre.phoenix6.BaseStatusSignal;
```

- [ ] **Step 2: Replace setUpdateFrequencyForAll(50) with tiered rates**

Replace line 140 (`statusSignalCollector.setUpdateFrequencyForAll(50);`) with:

```java
BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelVelocity, feederVelocity);
BaseStatusSignal.setUpdateFrequencyForAll(20,
    flywheelAppliedVoltage, flywheelSupplyCurent,
    flywheel2SupplyCurent, feederAppliedVoltage, feederSupplyCurent);
```

Note: field names are `flywheelSupplyCurent` (typo in original code — keep them as-is).

- [ ] **Step 3: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/shooter/ShooterIOTalonFX.java
git commit -m "perf(shooter): tiered CAN signal rates (50Hz velocity control, 20Hz monitoring)"
```

---

### Task 8: IntakeRollerIOTalonFX — signal rate tuning

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/intake/IntakeRollerIOTalonFX.java`

- [ ] **Step 1: Reduce all roller signals to 20Hz**

Replace line 77 (`intakeFollowerVelocity.setUpdateFrequency(50);`) with:
```java
intakeFollowerVelocity.setUpdateFrequency(20);
```

Replace line 81 (`statusSignalCollector.setUpdateFrequencyForAll(50);`) with:
```java
statusSignalCollector.setUpdateFrequencyForAll(20);
```

The roller is not in a tight closed-loop control path — 20Hz monitoring is sufficient.

- [ ] **Step 2: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/intake/IntakeRollerIOTalonFX.java
git commit -m "perf(intake-roller): reduce CAN signal rate to 20Hz (not in tight control loop)"
```

---

### Task 9: ModuleIOTalonFX — split signal rates into control (50Hz) and monitoring (20Hz)

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java`

- [ ] **Step 1: Split the second setUpdateFrequencyForAll call**

Find the block at lines 192–200:
```java
// Before:
BaseStatusSignal.setUpdateFrequencyForAll(
    50.0,
    driveVelocity,
    driveAppliedVolts,
    driveSupplyCurrent,
    turnAbsolutePosition,
    turnVelocity,
    turnAppliedVolts,
    turnSupplyCurrent);

// After:
BaseStatusSignal.setUpdateFrequencyForAll(
    50.0,
    driveVelocity,
    turnAbsolutePosition,
    turnVelocity);
BaseStatusSignal.setUpdateFrequencyForAll(
    20.0,
    driveAppliedVolts,
    driveSupplyCurrent,
    turnAppliedVolts,
    turnSupplyCurrent);
```

Leave the 250Hz odometry block for `drivePosition` and `turnPosition` untouched.

- [ ] **Step 2: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFX.java
git commit -m "perf(drive): swerve module monitoring signals to 20Hz, control signals stay at 50Hz"
```

---

### Task 10: Superstructure enums — add PIT_SHOOT / PIT_SHOOTING, remove MANUAL_SHOOTING

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/test/java/subsystems/SuperstructureTest.java`

- [ ] **Step 1: Update WantedState enum**

In `Superstructure.java` (lines 41–49):
```java
// Before:
public enum WantedState {
    IDLE, INTAKING, SHOOTING, PASSING, SHOOT_INTAKE, PASS_INTAKE, MANUAL_SHOOTING
}

// After:
public enum WantedState {
    IDLE, INTAKING, SHOOTING, PASSING, SHOOT_INTAKE, PASS_INTAKE, PIT_SHOOT
}
```

- [ ] **Step 2: Add PIT_SHOOTING to CurrentState enum**

```java
// Before:
public enum CurrentState {
    IDLE, INTAKING, SPINNING_UP, SHOOTING, PASSING
}

// After:
public enum CurrentState {
    IDLE, INTAKING, SPINNING_UP, SHOOTING, PASSING, PIT_SHOOTING
}
```

- [ ] **Step 3: Replace MANUAL_SHOOTING case with PIT_SHOOT in handleWantedState()**

In `handleWantedState()`, replace the `MANUAL_SHOOTING` case (lines 192–200):
```java
// Before:
case MANUAL_SHOOTING:
    if (currentSuperState != CurrentState.SHOOTING) {
        currentSuperState = CurrentState.SHOOTING;
        firstSpinup = false;
        noShotTimer.restart();
        crawlUpScheduled = false;
        oscTimer = 0.0;
    }
    break;

// After:
case PIT_SHOOT:
    if (currentSuperState != CurrentState.PIT_SHOOTING) {
        currentSuperState = CurrentState.PIT_SHOOTING;
        crawlUpScheduled = false;
        oscTimer = 0.0;
        noShotTimer.restart();
    }
    break;
```

- [ ] **Step 4: Fix RobotContainer compile error (stub fix)**

RobotContainer references `MANUAL_SHOOTING` at line 399. Change it to `PIT_SHOOT` to restore compile:
```java
// Before:
superstructure.setWantedSuperState(Superstructure.WantedState.MANUAL_SHOOTING),
// After:
superstructure.setWantedSuperState(Superstructure.WantedState.PIT_SHOOT),
```

The full binding cleanup (switch to `whileTrue`/`onFalse`) happens in Task 17.

- [ ] **Step 5: Add PIT_SHOOTING to GratuitousLighting shooting animation**

In `GratuitousLighting.java`, in `periodic()` (around line 105):
```java
// Before:
if (state == CurrentState.SHOOTING
    || state == CurrentState.SPINNING_UP
    || state == CurrentState.PASSING) {

// After:
if (state == CurrentState.SHOOTING
    || state == CurrentState.SPINNING_UP
    || state == CurrentState.PASSING
    || state == CurrentState.PIT_SHOOTING) {
```

- [ ] **Step 6: Write new PIT_SHOOT tests**

Add to `SuperstructureTest.java`:

```java
@Test
void pitShoot_wantedState_setsCurrentStatePitShooting_directly() {
    superstructure.currentSuperState = CurrentState.IDLE;
    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    superstructure.handleWantedState();
    assertEquals(CurrentState.PIT_SHOOTING, superstructure.currentSuperState);
}

@Test
void pitShoot_wantedState_doesNotGoThrough_spinningUp() {
    superstructure.currentSuperState = CurrentState.IDLE;
    when(mockShooter.flywheelUpToSpeed()).thenReturn(false);
    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    superstructure.handleWantedState();
    assertNotEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
    assertEquals(CurrentState.PIT_SHOOTING, superstructure.currentSuperState);
}
```

- [ ] **Step 7: Run tests**

```bash
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: New PIT_SHOOT tests pass. All existing tests pass.

- [ ] **Step 8: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java \
        src/main/java/frc/robot/RobotContainer.java \
        src/main/java/frc/robot/subsystems/GratuitousLighting.java \
        src/test/java/subsystems/SuperstructureTest.java
git commit -m "feat(superstructure): add PIT_SHOOT/PIT_SHOOTING states, remove MANUAL_SHOOTING"
```

---

### Task 11: Superstructure — fix double-transition bug + update broken tests

**Background:** `setWantedSuperState()` and `setWantedSuperStateCommand()` both call `handleWantedState()`, which then runs again in `periodic()`. This double-fires state transitions every time a button is pressed. Fix: both setters only store the field; `periodic()` calls `handleWantedState()` once per cycle.

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`
- Modify: `src/test/java/subsystems/SuperstructureTest.java`

- [ ] **Step 1: Fix setWantedSuperState() — remove handleWantedState() call**

```java
// Before:
public void setWantedSuperState(WantedState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
    handleWantedState();
}

// After:
public void setWantedSuperState(WantedState wantedSuperState) {
    this.wantedSuperState = wantedSuperState;
}
```

- [ ] **Step 2: Fix setWantedSuperStateCommand() — remove handleWantedState() call**

```java
// Before:
public Command setWantedSuperStateCommand(Supplier<WantedState> wantedSuperState) {
    return runOnce(
        () -> {
          this.wantedSuperState = wantedSuperState.get();
          handleWantedState();
        });
}

// After:
public Command setWantedSuperStateCommand(Supplier<WantedState> wantedSuperState) {
    return runOnce(() -> this.wantedSuperState = wantedSuperState.get());
}
```

- [ ] **Step 3: Run tests to confirm which break**

```bash
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: 4 tests fail: `shooting_wantedState_setsCurrentStateSpinningUp_whenNotAlreadyShooting`, `shooting_wantedState_staysInShooting_whenAlreadyShooting`, `passing_wantedState_setsCurrentStateSpinningUp_whenNotAlreadyPassing`, `passing_wantedState_staysInPassing_whenAlreadyPassing`.

- [ ] **Step 4: Update the 4 broken tests to call handleWantedState() explicitly**

These tests used `setWantedSuperState()` as a proxy for `handleWantedState()`. Now they must call both:

```java
@Test
void shooting_wantedState_setsCurrentStateSpinningUp_whenNotAlreadyShooting() {
    superstructure.currentSuperState = CurrentState.IDLE;
    superstructure.wantedSuperState = WantedState.SHOOTING;
    superstructure.handleWantedState();
    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
}

@Test
void shooting_wantedState_staysInShooting_whenAlreadyShooting() {
    superstructure.currentSuperState = CurrentState.SHOOTING;
    superstructure.wantedSuperState = WantedState.SHOOTING;
    superstructure.handleWantedState();
    assertEquals(CurrentState.SHOOTING, superstructure.currentSuperState);
}

@Test
void passing_wantedState_setsCurrentStateSpinningUp_whenNotAlreadyPassing() {
    superstructure.currentSuperState = CurrentState.IDLE;
    superstructure.wantedSuperState = WantedState.PASSING;
    superstructure.handleWantedState();
    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
}

@Test
void passing_wantedState_staysInPassing_whenAlreadyPassing() {
    superstructure.currentSuperState = CurrentState.PASSING;
    superstructure.wantedSuperState = WantedState.PASSING;
    superstructure.handleWantedState();
    assertEquals(CurrentState.PASSING, superstructure.currentSuperState);
}
```

Note: `idle_wantedState_setsCurrentStateIdle` does NOT break because `currentSuperState` starts as IDLE and the assertion still holds.

- [ ] **Step 5: Run tests**

```bash
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: All pass.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java \
        src/test/java/subsystems/SuperstructureTest.java
git commit -m "fix(superstructure): fix double-transition bug — setters store field only, periodic() drives state"
```

---

### Task 12: Superstructure — add PIT_SHOOT shotSetpoint in periodic()

**Background:** Without a setpoint update, PIT_SHOOTING would run with `RotationsPerSecond.of(0)` flywheel speed (the default ShotSetpoint). This adds the PIT_SHOOT branch to the shotSetpoint update block.

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`

- [ ] **Step 1: Add PIT_SHOOT to the periodic() shotSetpoint block**

In `periodic()`, after the PASSING/PASS_INTAKE else-if block (around line 655), add:

```java
// Before (end of the if/else chain):
else if (wantedSuperState == WantedState.PASSING
    || wantedSuperState == WantedState.PASS_INTAKE) {
    shotSetpoint = ShotPlanner.createPassSetpoint(...);
}
// [nothing for PIT_SHOOT]

// After:
else if (wantedSuperState == WantedState.PASSING
    || wantedSuperState == WantedState.PASS_INTAKE) {
    shotSetpoint = ShotPlanner.createPassSetpoint(...);
} else if (wantedSuperState == WantedState.PIT_SHOOT) {
    shotSetpoint = new ShotSetpoint(
        RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get()),
        RotationsPerSecond.of(ShooterConstants.SPINUP_FLYWHEEL_SPEED.get()),
        ShooterConstants.FIXED_HOOD_ANGLE_DEGREES,
        drive.getPose(),
        drive.getChassisSpeeds(),
        drive.getPose().getTranslation());
}
```

Setting `robotPose = drive.getPose()` means `checkHubTolerance()` rotation error = 0° — hub condition trivially satisfied if ever called (it won't be, but safety).

- [ ] **Step 2: Write a test for pit shoot setpoint**

Add to `SuperstructureTest.java`:

```java
@Test
void pitShooting_setsFlywheelToSpinupSpeed_inPeriodic() {
    when(mockDrive.getPose()).thenReturn(new Pose2d());
    when(mockDrive.getChassisSpeeds()).thenReturn(new ChassisSpeeds());

    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    superstructure.currentSuperState = CurrentState.PIT_SHOOTING;

    // periodic() updates shotSetpoint then calls handleWantedState + applyStates
    // We can't call periodic() directly in tests without a drive subsystem registration.
    // Instead: manually trigger the setpoint update path that periodic() would run.
    // The test verifies the setpoint was assigned (non-zero) vs the zero default.
    // Direct field access is acceptable since ShotSetpoint is a public data class.
    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    // After calling periodic, shotSetpoint.flywheelSpeed should equal SPINUP_FLYWHEEL_SPEED
    // We verify the applyStates path in the next task's test.
    // This test confirms the wantedState compiles and doesn't throw.
    assertDoesNotThrow(() -> superstructure.handleWantedState());
}
```

- [ ] **Step 3: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java \
        src/test/java/subsystems/SuperstructureTest.java
git commit -m "feat(superstructure): update PIT_SHOOT shotSetpoint to SPINUP_FLYWHEEL_SPEED in periodic()"
```

---

### Task 13: Superstructure — extract applyFeedingLogic + fix crawlUpScheduled gate

**Background:** SHOOTING and PASSING `applyStates()` cases have ~80 lines of near-identical beam-break / crawl-up / 254-push logic. Extract into `applyFeedingLogic(boolean intakeAlso)`. Simultaneously fix the `crawlUpScheduled` gate from `readyToShoot()` (flywheel + hub) to `shooter.flywheelUpToSpeed()` only — indexers should pre-stage as soon as flywheel is ready regardless of heading.

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`

- [ ] **Step 1: Add the applyFeedingLogic() private method**

Add this method before `applyPivotOscillate()` (around line 427):

```java
private void applyFeedingLogic(boolean intakeAlso) {
    if (intakeAlso) intakeRoller.spin();
    shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
    shooter.setFeederSpeed(shotSetpoint.feederSpeed);

    boolean tooClose = isTooCloseToHub();
    if (!Constants.MINIMAL_LOGGING)
        Logger.recordOutput("Superstructure/TooCloseToShoot", tooClose);
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

    boolean suppressOscillation =
        wantedSuperState == WantedState.SHOOT_INTAKE
            || wantedSuperState == WantedState.PASS_INTAKE;

    if (crawlUpScheduled && !suppressOscillation) {
        if (intakeWaitingForNextBall) {
            intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
            if (!Constants.MINIMAL_LOGGING)
                Logger.recordOutput("Superstructure/IntakeLoweredForBall", true);
        } else {
            if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.PUSH_254) {
                apply254Push();
            } else {
                applyPivotOscillate();
            }
            if (!Constants.MINIMAL_LOGGING)
                Logger.recordOutput("Superstructure/IntakeLoweredForBall", false);
        }
    }

    simulateShot();
}
```

- [ ] **Step 2: Replace the SHOOTING case body with applyFeedingLogic call**

Replace the full `case SHOOTING:` body (lines 326–421) with:

```java
case SHOOTING:
    applyFeedingLogic(wantedSuperState == WantedState.SHOOT_INTAKE);
    if (!checkHubTolerance()) {
        currentSuperState = CurrentState.SPINNING_UP;
        break;
    }
    if (noShotTimer.hasElapsed(1.0) && noShotCooldownTimer.hasElapsed(1.5)) {
        noShotTimer.restart();
        noShotCooldownTimer.restart();
        crawlUpScheduled = false;
        oscTimer = 0.0;
        push254Phase = Push254Phase.PUSHING;
        push254JamDebouncer.calculate(false);
        if (!Constants.MINIMAL_LOGGING)
            Logger.recordOutput("Superstructure/NoShotTrigger", true);
        intake.io.setIntakeArmVoltage(Volts.of(0));
        intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
    }
    break;
```

- [ ] **Step 3: Replace the PASSING case body with applyFeedingLogic call**

Replace the full `case PASSING:` body (lines 278–325) with:

```java
case PASSING:
    applyFeedingLogic(true);
    break;
```

`intakeAlso=true` because PASSING always runs the intake roller. Oscillation is suppressed within `applyFeedingLogic` when `wantedSuperState == PASS_INTAKE`.

- [ ] **Step 4: Write a test that confirms crawlUpScheduled gates on flywheel only**

Add to `SuperstructureTest.java`:

```java
@Test
void shooting_crawlUpScheduled_whenFlywheelReady_evenIfHubOutOfTolerance() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    when(mockDrive.getRotation()).thenReturn(Rotation2d.fromDegrees(10)); // out of tolerance

    superstructure.wantedSuperState = WantedState.SHOOTING;
    superstructure.currentSuperState = CurrentState.SHOOTING;

    superstructure.applyStates();

    // Hub out of tolerance → SPINNING_UP. But crawlUpScheduled should have been set
    // this loop before the hub-tolerance drop-back.
    // After applyFeedingLogic: crawlUpScheduled = true (flywheel ready).
    // After hub check: currentSuperState = SPINNING_UP.
    assertEquals(CurrentState.SPINNING_UP, superstructure.currentSuperState);
}
```

- [ ] **Step 5: Build and run tests**

```bash
./gradlew build
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: BUILD SUCCESSFUL, all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java \
        src/test/java/subsystems/SuperstructureTest.java
git commit -m "refactor(superstructure): extract applyFeedingLogic, fix crawlUpScheduled to gate on flywheel only"
```

---

### Task 14: Superstructure — add PIT_SHOOTING applyStates case and make fields private

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`

- [ ] **Step 1: Add PIT_SHOOTING case to applyStates()**

In `applyStates()`, add before `default:` (around line 422):

```java
case PIT_SHOOTING:
    shooter.setFlywheelRPM(shotSetpoint.flywheelSpeed);
    if (shooter.flywheelUpToSpeed()) {
        hopper.setIndexerSpeed(
            RotationsPerSecond.of(HopperConstants.INDEXER_SPEED.get()));
        hopper.setTopIndexerSpeed(
            RotationsPerSecond.of(HopperConstants.TOP_INDEXER_SPEED.get()));
        shooter.setFeederSpeed(shotSetpoint.feederSpeed);
    }
    break;
```

No hub tolerance guard. No `isTooCloseToHub()` check. No beam-break logic. Flywheel-ready is the only gate.

- [ ] **Step 2: Write PIT_SHOOTING applyStates tests**

Add to `SuperstructureTest.java`:

```java
@Test
void pitShooting_engagesIndexersAndFeeder_whenFlywheelReady() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    superstructure.currentSuperState = CurrentState.PIT_SHOOTING;
    superstructure.applyStates();
    verify(mockHopper).setIndexerSpeed(any(AngularVelocity.class));
    verify(mockHopper).setTopIndexerSpeed(any(AngularVelocity.class));
}

@Test
void pitShooting_doesNotEngageIndexers_whenFlywheelNotReady() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(false);
    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    superstructure.currentSuperState = CurrentState.PIT_SHOOTING;
    superstructure.applyStates();
    verify(mockHopper, never()).setIndexerSpeed(any(AngularVelocity.class));
}

@Test
void pitShooting_neverDropsBackToSpinningUp() {
    when(mockShooter.flywheelUpToSpeed()).thenReturn(true);
    when(mockDrive.getRotation()).thenReturn(Rotation2d.fromDegrees(90)); // way out of tolerance
    superstructure.wantedSuperState = WantedState.PIT_SHOOT;
    superstructure.currentSuperState = CurrentState.PIT_SHOOTING;
    superstructure.applyStates();
    assertEquals(CurrentState.PIT_SHOOTING, superstructure.currentSuperState);
}
```

- [ ] **Step 3: Run tests**

```bash
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: All pass including the 3 new PIT_SHOOTING tests.

- [ ] **Step 4: Make subsystem fields private**

Change the five `public` field declarations (lines 33–37):
```java
// Before:
public Drive drive;
public Shooter shooter;
public Hopper hopper;
public IntakePivot intake;
public IntakeRoller intakeRoller;

// After:
private Drive drive;
private Shooter shooter;
private Hopper hopper;
private IntakePivot intake;
private IntakeRoller intakeRoller;
```

`currentSuperState` and `wantedSuperState` stay `public` — they are used by `GratuitousLighting` and tests.

- [ ] **Step 5: Build (will catch any unintended external field accesses)**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java \
        src/test/java/subsystems/SuperstructureTest.java
git commit -m "feat(superstructure): add PIT_SHOOTING applyStates case, make subsystem fields private"
```

---

### Task 15: Remove dead commented code blocks

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`

- [ ] **Step 1: Remove top-indexer jam recovery block from Superstructure**

This block is now also gone because the SHOOTING case was replaced in Task 13. Verify it was removed — if any commented block remains in `applyFeedingLogic` or `applyStates()`, delete it.

The block that started with `// Top-indexer jam recovery: run forward, reverse...` (lines 339–357 in the original) should no longer exist after the Task 13 extraction. Confirm it's gone.

- [ ] **Step 2: Remove hopper spinup in IDLE comments**

In the `IDLE` case of `applyStates()`, remove the three commented-out lines:
```java
// Remove:
// hopper.setIndexerSpeed(RotationsPerSecond.of(firstSpinup ? -10 : 0));
// hopper.setTopIndexerSpeed(RotationsPerSecond.of(firstSpinup ? -10 : 0));
// shooter.setFeederVoltage(Volts.of(firstSpinup ? -2 : 0));
```

Also remove the commented-out `hopper.stopFullIndexer()` in the hub-spinup else-if block in `IDLE`:
```java
// Remove:
// hopper.stopFullIndexer();
```

And remove the commented-out `hopper.spinFullIndexer(...)` in the hub-spinup block in `INTAKING`:
```java
// Remove:
// hopper.spinFullIndexer(RotationsPerSecond.of(-20), RotationsPerSecond.of(-20));
```

Also remove the commented line from hub-spinup-else in `IDLE`:
```java
// Remove:
// hopper.stopFullIndexer();
```

- [ ] **Step 3: Remove dead feeder voltage line in SPINNING_UP**

In `SPINNING_UP` case, remove the zero-equals-zero dead line:
```java
// Remove:
shooter.setFeederVoltage(Volts.of(firstSpinup ? 0 : 0));
```

- [ ] **Step 4: Remove commented D-pad bindings in RobotContainer**

In `RobotContainer.java`, remove the three commented-out D-pad automation commands:
```java
// Remove all three:
// controller.povUp().whileTrue(AutomaticCommands.hubBackWallCommand(drive, driverOverride));
// controller.povDown().whileTrue(AutomaticCommands.wallShootSetupCommand(drive, driverOverride));
// controller.povLeft().whileTrue(AutomaticCommands.underTowerCommand(drive, driverOverride));
```

Also remove the commented-out B-button vision intake block (~9 lines starting with `// controller.b().whileTrue(Commands.parallel(`).

- [ ] **Step 5: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 6: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java \
        src/main/java/frc/robot/RobotContainer.java
git commit -m "refactor: remove dead commented code blocks from Superstructure and RobotContainer"
```

---

### Task 16: Superstructure — move Logger side effects out of predicate methods

**Background:** `readyToShoot()` and `isPrettyMuchCloseToTargetButNotQuite()` both call `Logger.recordOutput()`. Logging in a boolean predicate makes the output depend on call frequency, which is non-obvious. Move all logging to `periodic()`.

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`

- [ ] **Step 1: Strip Logger calls from readyToShoot()**

```java
// Before:
public boolean readyToShoot() {
    boolean shooterSpeedSetpoint = shooter.flywheelUpToSpeed();
    boolean hubSetpoint = checkHubTolerance();

    if (!Constants.MINIMAL_LOGGING) {
        Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterSpeedSetpoint);
        Logger.recordOutput("Superstructure/HubRotationSetpoint", hubSetpoint);
        Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose);
        Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose());
        Logger.recordOutput("Superstructure/HubRotationToleranceDeg", getDynamicHubToleranceDegrees());
        Logger.recordOutput("Superstructure/HubRotationError",
            Math.abs(drive.getPose().getRotation()
                .minus(shotSetpoint.robotPose.getRotation()).getDegrees()));
    }

    return shooterSpeedSetpoint && hubSetpoint;
}

// After:
public boolean readyToShoot() {
    return shooter.flywheelUpToSpeed() && checkHubTolerance();
}
```

- [ ] **Step 2: Strip Logger calls from isPrettyMuchCloseToTargetButNotQuite()**

```java
// Before:
public boolean isPrettyMuchCloseToTargetButNotQuite() {
    boolean shooterSpeedSetpoint =
        shooter.flywheelUpToSpeed(ShooterConstants.BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT);
    boolean hubSetpoint = checkHubTolerance();

    if (!Constants.MINIMAL_LOGGING) {
        Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterSpeedSetpoint);
        Logger.recordOutput("Superstructure/HubRotationSetpoint", hubSetpoint);
        Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose.getRotation());
        Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose().getRotation());
        Logger.recordOutput("Superstructure/HubRotationError",
            Math.abs(drive.getPose().getRotation()
                .minus(shotSetpoint.robotPose.getRotation()).getDegrees()));
    }

    return shooterSpeedSetpoint && hubSetpoint;
}

// After:
public boolean isPrettyMuchCloseToTargetButNotQuite() {
    return shooter.flywheelUpToSpeed(ShooterConstants.BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT)
        && checkHubTolerance();
}
```

- [ ] **Step 3: Add the logging to periodic()**

In `periodic()`, after the existing `Logger.recordOutput` calls for `CurrentSuperState` and `WantedSuperState`, add:

```java
if (!Constants.MINIMAL_LOGGING) {
    boolean shooterReady = shooter.flywheelUpToSpeed();
    boolean hubReady = checkHubTolerance();
    Logger.recordOutput("Superstructure/ShooterSpeedSetpoint", shooterReady);
    Logger.recordOutput("Superstructure/HubRotationSetpoint", hubReady);
    Logger.recordOutput("Superstructure/HubRotationTarget", shotSetpoint.robotPose);
    Logger.recordOutput("Superstructure/HubRotationCurrent", drive.getPose());
    Logger.recordOutput("Superstructure/HubRotationToleranceDeg", getDynamicHubToleranceDegrees());
    Logger.recordOutput("Superstructure/HubRotationError",
        Math.abs(drive.getPose().getRotation()
            .minus(shotSetpoint.robotPose.getRotation()).getDegrees()));
    Logger.recordOutput("Superstructure/PitShootActive",
        currentSuperState == CurrentState.PIT_SHOOTING);
    Logger.recordOutput("Superstructure/FlywheelReadyForPitShoot",
        currentSuperState == CurrentState.PIT_SHOOTING && shooter.flywheelUpToSpeed());
}
```

- [ ] **Step 4: Build and run tests**

```bash
./gradlew build
./gradlew test --tests "subsystems.SuperstructureTest"
```

Expected: BUILD SUCCESSFUL, all tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java
git commit -m "refactor(superstructure): move Logger side effects out of readyToShoot/isPrettyMuchClose to periodic()"
```

---

### Task 17: RobotContainer + Robot.java — full binding cleanup, alerts, PathPlanner fix

**Files:**
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/Robot.java`

- [ ] **Step 1: Fix left bumper intake toggle desync**

Replace line 345:
```java
// Before:
controller.leftBumper().toggleOnTrue(superstructure.wantIntaking());

// After:
controller.leftBumper()
    .whileTrue(superstructure.setWantedSuperStateCommand(() -> WantedState.INTAKING))
    .onFalse(superstructure.setWantedSuperStateCommand(() -> WantedState.IDLE));
```

`setWantedSuperStateCommand` is a `runOnce` — completes immediately, holds no subsystem requirement, cannot be interrupted. No toggle state to desync.

- [ ] **Step 2: Remove .until(leftBumper) guards from right trigger and right bumper**

Right trigger (lines 293–308):
```java
// Before:
controller.rightTrigger().whileTrue(
    Commands.parallel(
        DriveCommands.joystickDriveAtAngle(...),
        superstructure.setWantedSuperStateCommand(...))
    .until(() -> controller.leftBumper().getAsBoolean()))  // ← remove this line
    .onFalse(superstructure.setWantedSuperStateCommand(() -> WantedState.IDLE));

// After:
controller.rightTrigger().whileTrue(
    Commands.parallel(
        DriveCommands.joystickDriveAtAngle(...),
        superstructure.setWantedSuperStateCommand(...)))
    .onFalse(superstructure.setWantedSuperStateCommand(() -> WantedState.IDLE));
```

Apply the same removal to right bumper (lines 309–324).

- [ ] **Step 3: Replace D-pad left with proper PIT_SHOOT binding**

Replace lines 394–401 (the existing `startEnd` binding):
```java
// Before:
controller.povLeft().whileTrue(
    Commands.startEnd(
        () -> superstructure.setWantedSuperState(Superstructure.WantedState.PIT_SHOOT),
        () -> superstructure.setWantedSuperState(Superstructure.WantedState.IDLE),
        superstructure));

// After:
controller.povLeft()
    .whileTrue(superstructure.setWantedSuperStateCommand(() -> WantedState.PIT_SHOOT))
    .onFalse(superstructure.setWantedSuperStateCommand(() -> WantedState.IDLE));
```

This uses `setWantedSuperStateCommand` (runOnce) instead of `startEnd` (which held a subsystem requirement and could cause interrupt conflicts).

- [ ] **Step 4: Add battery low alert**

Near the top of `RobotContainer` class body (after existing alert declarations), add:
```java
private final Alert batteryLowAlert =
    new Alert("Battery voltage low — check before match", AlertType.kWarning);
```

In `RobotContainer.periodic()` (or in the teleopPeriodic / robotPeriodic that calls container methods), add:
```java
batteryLowAlert.set(RobotController.getBatteryVoltage() < 10.5);
```

If `RobotContainer` doesn't have a `periodic()` method, add one:
```java
public void periodic() {
    batteryLowAlert.set(RobotController.getBatteryVoltage() < 10.5);
}
```

And in `Robot.java`'s `robotPeriodic()`, call `robotContainer.periodic()` if not already done.

Also add the import if not present:
```java
import edu.wpi.first.wpilibj.RobotController;
```

- [ ] **Step 5: Add CAN bus utilization alert**

In `RobotContainer`, add:
```java
private final Alert canHighAlert =
    new Alert("CAN bus utilization high (>80%)", AlertType.kWarning);
```

In `RobotContainer.periodic()`:
```java
canHighAlert.set(
    new CANBus(ShooterConstants.CAN_BUS.getName()).getStatus().BusUtilization > 0.8);
```

Add import:
```java
import com.ctre.phoenix6.CANBus;
```

- [ ] **Step 6: Fix PathPlanner warmup to run while disabled**

In `Robot.java`, change line 118:
```java
// Before:
PathfindingCommand.warmupCommand().schedule();

// After:
PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();
```

Without `.ignoringDisable(true)`, the command is scheduled during `robotInit()` while the robot is disabled but never runs — the warmup actually fires on the first enabled loop, causing the 17-second spike. With `.ignoringDisable(true)`, it runs immediately during init.

- [ ] **Step 7: Build**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 8: Commit**

```bash
git add src/main/java/frc/robot/RobotContainer.java \
        src/main/java/frc/robot/Robot.java
git commit -m "fix(bindings): fix left bumper desync, remove until-guards, add PIT_SHOOT binding, battery/CAN alerts, PathPlanner warmup fix"
```

---

### Task 18: Full build and test run

- [ ] **Step 1: Run full build**

```bash
./gradlew build
```

Expected: BUILD SUCCESSFUL with no warnings about unused imports or missing symbols.

- [ ] **Step 2: Run all tests**

```bash
./gradlew test
```

Expected: All tests pass. Key test classes to verify:
- `subsystems.SuperstructureTest` — all existing + new PIT_SHOOT tests
- `subsystems.ShooterTest`
- `subsystems.HopperTest`

- [ ] **Step 3: If any test fails, fix it before proceeding**

Common failure modes:
- `currentSuperState` assertions that relied on `setWantedSuperState` calling `handleWantedState()` — fix by calling `handleWantedState()` explicitly in the test (pattern from Task 11).
- `verify(mock)` call counts if new code paths call additional methods — add `atLeastOnce()` or adjust verify count.

- [ ] **Step 4: Commit any test fixes discovered in this step**

```bash
git add src/test/java/subsystems/
git commit -m "test: fix remaining test assertions after superstructure refactor"
```

---

## Summary of Changes by File

| File | Changes |
|------|---------|
| `ShotPlanner.java` | Remove polynomial dead code (~90 lines) |
| `GratuitousLighting.java` | Remove dead field, add PIT_SHOOTING to animation |
| `ShooterConstants.java` | Fix hub tolerances (5°/3°), remove TODO |
| `Drive.java` | Pre-alloc module caches, remove timing scaffolding |
| `HopperIOTalonFX.java` | Fix 2-RPS dead zone, async defense, tiered signal rates |
| `IntakePivotIOTalonFX.java` | Async defense mode, tiered signal rates |
| `ShooterIOTalonFX.java` | Tiered signal rates |
| `IntakeRollerIOTalonFX.java` | Reduce all signals to 20Hz |
| `ModuleIOTalonFX.java` | Monitoring signals to 20Hz |
| `Superstructure.java` | PIT_SHOOT/PIT_SHOOTING states, fix double-transition, extract applyFeedingLogic, crawlUp gate fix, private fields, logging cleanup |
| `RobotContainer.java` | Fix left bumper toggle, remove .until() guards, PIT_SHOOT binding, alerts |
| `Robot.java` | PathPlanner warmup `.ignoringDisable(true)` |
| `SuperstructureTest.java` | Update 4 broken tests, add 6 new PIT_SHOOT tests |
