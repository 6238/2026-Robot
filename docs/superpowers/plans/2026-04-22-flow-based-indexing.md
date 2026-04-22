# Flow-Based Indexing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `BEAM_HOPPER` and `FUSED_HOPPER` indexing modes that skip the post-shot arm-drop when balls are already queued in the hopper, increasing shot throughput.

**Architecture:** Two new `IndexingMode` values route `applyFeedingLogic()` to new methods that use a beam-break flow window (and optionally indexer current) to decide whether to keep oscillating or drop the arm. The existing `intakeWaitingForNextBall` arm-drop logic is gated behind `!isFlowMode` so all existing modes are unchanged.

**Tech Stack:** WPILib Command-based Java, AdvantageKit logging, CTRE TalonFX, JUnit 5 + Mockito 5

---

## File Map

| File | Change |
|---|---|
| `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java` | Add `BEAM_HOPPER`, `FUSED_HOPPER` to `IndexingMode`; add `FLOW_WINDOW_SECONDS`, `FUSED_JAM_CURRENT_AMPS` constants |
| `src/main/java/frc/robot/subsystems/shooter/Shooter.java` | Add `getTimeSinceLastBeamBreakSec()` public method |
| `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java` | Gate `intakeWaitingForNextBall` behind `!isFlowMode`; add `applyBeamHopperFeeding()`, `applyFusedHopperFeeding()`; extend mode dispatch in `applyFeedingLogic()` |
| `src/test/java/subsystems/ShooterTest.java` | Tests for `getTimeSinceLastBeamBreakSec()` |

---

## Task 1: TDD — `getTimeSinceLastBeamBreakSec()` on Shooter

**Files:**
- Modify: `src/test/java/subsystems/ShooterTest.java`
- Modify: `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

- [ ] **Step 1.1: Write the failing tests**

Add to `ShooterTest.java` after the `getCurrentFlywheelSpeed` block:

```java
// ── getTimeSinceLastBeamBreakSec ──────────────────────────────────────────

@Test
void getTimeSinceLastBeamBreakSec_returnsMaxValueBeforeAnyBeamBreak() {
  assertEquals(Double.MAX_VALUE, shooter.getTimeSinceLastBeamBreakSec());
}

@Test
void getTimeSinceLastBeamBreakSec_returnsNonNegativeAfterBeamBreakFires() {
  doAnswer(
          inv -> {
            ShooterIOInputs inputs = inv.getArgument(0);
            inputs.flywheelTalonConnected = true;
            inputs.feederTalonConnected = true;
            inputs.beamBreakTriggered = true;
            return null;
          })
      .when(mockShooterIO)
      .updateInputs(any(ShooterIOInputs.class));
  shooter.periodic();

  double elapsed = shooter.getTimeSinceLastBeamBreakSec();
  assertTrue(elapsed >= 0.0, "elapsed should be non-negative after beam break");
  assertTrue(elapsed < 1.0, "elapsed should be very small immediately after beam break");
}

@Test
void getTimeSinceLastBeamBreakSec_stillMaxValueWhenBeamBreakNotTriggered() {
  // periodic called but beam break never fires
  shooter.periodic();
  assertEquals(Double.MAX_VALUE, shooter.getTimeSinceLastBeamBreakSec());
}
```

- [ ] **Step 1.2: Run tests to verify they fail**

```bash
./gradlew test --tests "subsystems.ShooterTest.getTimeSinceLastBeamBreakSec_*"
```

Expected: FAIL — `getTimeSinceLastBeamBreakSec` method does not exist.

- [ ] **Step 1.3: Implement the method in Shooter.java**

Add after the `isShooting()` method (around line 112):

```java
/** Seconds since the beam break last fired; {@link Double#MAX_VALUE} if never triggered. */
public double getTimeSinceLastBeamBreakSec() {
  if (lastBeamBreakTimestampSec < 0) return Double.MAX_VALUE;
  return Timer.getFPGATimestamp() - lastBeamBreakTimestampSec;
}
```

- [ ] **Step 1.4: Run tests to verify they pass**

```bash
./gradlew test --tests "subsystems.ShooterTest.getTimeSinceLastBeamBreakSec_*"
```

Expected: All 3 tests PASS.

- [ ] **Step 1.5: Commit**

```bash
git add src/test/java/subsystems/ShooterTest.java src/main/java/frc/robot/subsystems/shooter/Shooter.java
git commit -m "feat(shooter): add getTimeSinceLastBeamBreakSec()"
```

---

## Task 2: Add BEAM_HOPPER and FUSED_HOPPER constants

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`

- [ ] **Step 2.1: Add enum values**

Find the `IndexingMode` enum (currently lines 16–19):

```java
public enum IndexingMode {
  OSCILLATE,
  PUSH_254
}
```

Replace with:

```java
public enum IndexingMode {
  OSCILLATE,
  PUSH_254,
  BEAM_HOPPER,
  FUSED_HOPPER
}
```

- [ ] **Step 2.2: Add tunable constants**

Find `public static final IndexingMode INDEXING_MODE = IndexingMode.OSCILLATE;` (line 21). Add the two new constants directly below it:

```java
public static final IndexingMode INDEXING_MODE = IndexingMode.OSCILLATE;

public static final LoggedNetworkNumber FLOW_WINDOW_SECONDS =
    new LoggedNetworkNumber("Shooter/FlowWindowSeconds", 0.4);

public static final LoggedNetworkNumber FUSED_JAM_CURRENT_AMPS =
    new LoggedNetworkNumber("Shooter/FusedJamCurrentAmps", 8.0);
```

- [ ] **Step 2.3: Build to verify no compile errors**

```bash
./gradlew build -x test
```

Expected: BUILD SUCCESSFUL

- [ ] **Step 2.4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java
git commit -m "feat(shooter): add BEAM_HOPPER, FUSED_HOPPER IndexingMode and flow constants"
```

---

## Task 3: Implement applyBeamHopperFeeding() and wire into applyFeedingLogic()

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`

- [ ] **Step 3.1: Add applyBeamHopperFeeding() method**

Add this new private method directly before the existing `apply254Push()` method:

```java
private void applyBeamHopperFeeding() {
  double timeSinceShot = shooter.getTimeSinceLastBeamBreakSec();
  boolean flowActive = timeSinceShot < ShooterConstants.FLOW_WINDOW_SECONDS.get();

  if (flowActive) {
    applyPivotOscillate();
  } else {
    intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
  }

  if (!Constants.MINIMAL_LOGGING) {
    Logger.recordOutput("Superstructure/FlowActive", flowActive);
    Logger.recordOutput("Superstructure/TimeSinceLastShot", timeSinceShot);
  }
}
```

- [ ] **Step 3.2: Gate intakeWaitingForNextBall behind !isFlowMode in applyFeedingLogic()**

Find the full `applyFeedingLogic()` method and replace it with the version below. The key changes are:
1. `isFlowMode` computed near the top
2. The `intakeWaitingForNextBall` rising-edge and timer blocks wrapped in `if (!isFlowMode)`
3. The mode dispatch extended with an `else if` for `BEAM_HOPPER`
4. The arm-drop guard changed from `intakeWaitingForNextBall` to `!isFlowMode && intakeWaitingForNextBall`

```java
private boolean applyFeedingLogic() {
  if (shooter.flywheelUpToSpeed() && !crawlUpScheduled) {
    crawlUpScheduled = true;
    oscTimer = 0.0;
    push254Phase = Push254Phase.PUSHING;
    push254JamDebouncer.calculate(false);
  }

  boolean beamBreak = shooter.isShooting();
  boolean beamBreakRisingEdge = beamBreak && !prevBeamBreak;

  boolean isFlowMode =
      ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.BEAM_HOPPER
          || ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.FUSED_HOPPER;

  if (!isFlowMode) {
    if (beamBreakRisingEdge) {
      intakeWaitingForNextBall = !intakeWaitingForNextBall;
      if (intakeWaitingForNextBall) {
        intakeDownTimer.restart();
        push254Phase = Push254Phase.PUSHING;
        push254JamDebouncer.calculate(false);
      }
    }
    if (intakeWaitingForNextBall && intakeDownTimer.hasElapsed(0.25)) {
      intakeWaitingForNextBall = false;
      push254Phase = Push254Phase.PUSHING;
      push254JamDebouncer.calculate(false);
    }
  }
  prevBeamBreak = beamBreak;

  if (crawlUpScheduled
      && wantedSuperState != WantedState.SHOOT_INTAKE
      && wantedSuperState != WantedState.PASS_INTAKE) {
    if (!isFlowMode && intakeWaitingForNextBall) {
      intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
      if (!Constants.MINIMAL_LOGGING)
        Logger.recordOutput("Superstructure/IntakeLoweredForBall", true);
    } else {
      if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.PUSH_254) {
        apply254Push();
      } else if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.BEAM_HOPPER) {
        applyBeamHopperFeeding();
      } else if (ShooterConstants.INDEXING_MODE == ShooterConstants.IndexingMode.FUSED_HOPPER) {
        applyFusedHopperFeeding();
      } else {
        applyPivotOscillate();
      }
      if (!Constants.MINIMAL_LOGGING)
        Logger.recordOutput("Superstructure/IntakeLoweredForBall", false);
    }
  }

  simulateShot();
  return beamBreakRisingEdge;
}
```

- [ ] **Step 3.3: Build and run all tests**

```bash
./gradlew build
```

Expected: BUILD SUCCESSFUL, all tests pass. If `applyFusedHopperFeeding()` is not yet defined, there will be a compile error — that is expected and resolved in Task 4.

Actually: add a stub so it compiles now. Add this placeholder directly below `applyBeamHopperFeeding()`:

```java
private void applyFusedHopperFeeding() {
  applyBeamHopperFeeding();
}
```

Then re-run:

```bash
./gradlew build
```

Expected: BUILD SUCCESSFUL, all tests pass.

- [ ] **Step 3.4: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java
git commit -m "feat(superstructure): add BEAM_HOPPER flow-based indexing mode"
```

---

## Task 4: Implement applyFusedHopperFeeding()

**Files:**
- Modify: `src/main/java/frc/robot/subsystems/superstructure/Superstructure.java`

- [ ] **Step 4.1: Replace the stub with the real implementation**

Find the stub `applyFusedHopperFeeding()` added in Task 3 and replace it with:

```java
private void applyFusedHopperFeeding() {
  double timeSinceShot = shooter.getTimeSinceLastBeamBreakSec();
  boolean flowActive = timeSinceShot < ShooterConstants.FLOW_WINDOW_SECONDS.get();
  boolean hopperJammed =
      hopper.inputs.indexerSupplyCurrent.in(Amps)
          > ShooterConstants.FUSED_JAM_CURRENT_AMPS.get();

  if (flowActive && !hopperJammed) {
    applyPivotOscillate();
  } else {
    intake.setAngle(Degrees.of(IntakeConstants.INTAKE_DOWN_VALUE.get()));
  }

  if (!Constants.MINIMAL_LOGGING) {
    Logger.recordOutput("Superstructure/FlowActive", flowActive);
    Logger.recordOutput("Superstructure/TimeSinceLastShot", timeSinceShot);
    Logger.recordOutput("Superstructure/HopperJammed", hopperJammed);
  }
}
```

`Amps` is already available via the `import static edu.wpi.first.units.Units.*;` at the top of Superstructure.java.

- [ ] **Step 4.2: Build and run all tests**

```bash
./gradlew build
```

Expected: BUILD SUCCESSFUL, all tests pass.

- [ ] **Step 4.3: Commit**

```bash
git add src/main/java/frc/robot/subsystems/superstructure/Superstructure.java
git commit -m "feat(superstructure): add FUSED_HOPPER flow+current-based indexing mode"
```

---

## Activation

To try `BEAM_HOPPER` mode, change line 21 of `ShooterConstants.java`:

```java
public static final IndexingMode INDEXING_MODE = IndexingMode.BEAM_HOPPER;
```

To try `FUSED_HOPPER`:

```java
public static final IndexingMode INDEXING_MODE = IndexingMode.FUSED_HOPPER;
```

Tune `Shooter/FlowWindowSeconds` (default 0.4s) and `Shooter/FusedJamCurrentAmps` (default 8.0A) via NetworkTables/Tuning mode. Check logs for `Superstructure/FlowActive`, `Superstructure/TimeSinceLastShot`, and `Superstructure/HopperJammed` to validate behavior.

> **FUSED_HOPPER calibration note:** Verify normal indexer running current in logs before setting `FUSED_JAM_CURRENT_AMPS`. The threshold must be above steady-state running current or it will false-positive and drop the arm constantly.
