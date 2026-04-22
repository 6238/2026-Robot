# Flow-Based Indexing Design

**Date:** 2026-04-22
**Branch:** Optimization

## Problem

During shooting, the arm drops to -2.5° for 0.25s after every beam break rising edge (`intakeWaitingForNextBall` logic). Balls are already queued in the hopper — not on the ground — so this drop is wasted dead time every shot cycle.

## Goal

Two new indexing modes that eliminate unnecessary arm drops by using sensor feedback to decide whether the hopper is actually empty or jammed before dropping the arm.

---

## New IndexingMode Values

Add to `ShooterConstants.IndexingMode`:

```java
BEAM_HOPPER,   // beam-break flow window only
FUSED_HOPPER   // flow window + indexer current jam veto
```

`INDEXING_MODE` stays a compile-time constant (same as today).

---

## BEAM_HOPPER

**Heuristic:** While balls have been flowing recently, keep oscillating. Only drop the arm when flow has been quiet long enough to infer the hopper is empty.

**Logic (per loop):**

```
flowActive = shooter.getTimeSinceLastBeamBreakSec() < FLOW_WINDOW_SECONDS

if crawlUpScheduled:
    if flowActive:
        applyPivotOscillate()       // no arm drop, keep pushing
    else:
        intake.setAngle(INTAKE_DOWN_VALUE)   // quiet = reload
```

`intakeWaitingForNextBall` is not used in this mode.

**Tunable constant:**

| Constant | Type | Default |
|---|---|---|
| `FLOW_WINDOW_SECONDS` | `LoggedNetworkNumber` | `0.4` |

---

## FUSED_HOPPER

**Heuristic:** Same as BEAM_HOPPER, but indexer current spikes veto the "stay up" decision — high current means the hopper is jammed and adding more arm pressure is counterproductive.

**Logic (per loop):**

```
flowActive   = shooter.getTimeSinceLastBeamBreakSec() < FLOW_WINDOW_SECONDS
hopperJammed = hopper.inputs.indexerSupplyCurrent > FUSED_JAM_CURRENT_AMPS

if crawlUpScheduled:
    if flowActive AND NOT hopperJammed:
        applyPivotOscillate()
    else:
        intake.setAngle(INTAKE_DOWN_VALUE)   // jammed OR quiet = drop arm
```

`hopperJammed` drops the arm because the hopper needs pressure relieved, not increased.

**Tunable constants:**

| Constant | Type | Default |
|---|---|---|
| `FLOW_WINDOW_SECONDS` | `LoggedNetworkNumber` | `0.4` |
| `FUSED_JAM_CURRENT_AMPS` | `LoggedNetworkNumber` | `8.0` |

---

## Implementation Changes

### `ShooterConstants.java`
- Add `BEAM_HOPPER`, `FUSED_HOPPER` to `IndexingMode` enum
- Add `FLOW_WINDOW_SECONDS` (`LoggedNetworkNumber`, default 0.4)
- Add `FUSED_JAM_CURRENT_AMPS` (`LoggedNetworkNumber`, default 8.0)

### `Shooter.java`
- Add `getTimeSinceLastBeamBreakSec()`: returns `Timer.getFPGATimestamp() - lastBeamBreakTimestampSec`, or `Double.MAX_VALUE` if never triggered

### `Superstructure.java`
- In `applyFeedingLogic()`: add `case BEAM_HOPPER` and `case FUSED_HOPPER` branches alongside the existing `PUSH_254` / `OSCILLATE` paths
- New private methods: `applyBeamHopperFeeding()`, `applyFusedHopperFeeding()`
- Both methods skip `intakeWaitingForNextBall` entirely — that state variable is only used by OSCILLATE/PUSH_254

### What does NOT change
- `crawlUpScheduled` flag and its conditions
- No-shot 1s recovery timer
- `SHOOT_INTAKE` / `PASS_INTAKE` roller spin path
- `intakeWaitingForNextBall` variable (kept, only inactive for new modes)
- All existing OSCILLATE and PUSH_254 behavior

---

## Logging

Both new methods should log:

```
Superstructure/FlowActive           boolean
Superstructure/TimeSinceLastShot    double (seconds)
Superstructure/HopperJammed         boolean   // FUSED_HOPPER only
```

---

## Tuning Notes

- Start `FLOW_WINDOW_SECONDS` at 0.4s — covers one slow shot cycle without being so long that an empty hopper keeps the arm up
- `FUSED_JAM_CURRENT_AMPS` at 8.0A is a starting point; check logs for normal indexer running current first (baseline may already exceed 8A under load), then calibrate against a known jam event — threshold must be above normal running current or it will false-positive constantly
