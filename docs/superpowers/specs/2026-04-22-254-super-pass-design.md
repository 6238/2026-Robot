# 254 Super Pass Implementation Design

> **For agentic workers:** Use superpowers:writing-plans to turn this spec into an implementation plan.

**Goal:** Add a D-Pad Up teleop command that pathfinds to the start of a preset pass route and follows it, automatically mirrored for alliance and field half.

**Architecture:** Load one of two path files based on robot position (top/bottom half), apply PathPlanner's built-in alliance flip for red, then use `pathfindThenFollowPath`. Pure navigation — no superstructure state.

---

## Path Files

Two `.path` files live in `src/main/deploy/pathplanner/paths/`:

- **`Elliot is Special.path`** — existing, defined for blue alliance, bottom half. Starts at (5.856, 3.315), sweeps through neutral/red zone near the bottom wall, ends at (6.025, 0.627).
- **`Upward Elliot is Special.path`** — new file, horizontal mirror of `Elliot is Special.path`. Every Y coordinate is replaced with `8.21 - Y`. Start/end rotation angles are negated. No rotation targets or constraint zones to transform.

### Upward Elliot is Special computed values (field width = 8.21 m)

**Waypoints:**

| # | anchor X | anchor Y | prevControl X | prevControl Y | nextControl X | nextControl Y |
|---|----------|----------|---------------|---------------|---------------|---------------|
| 1 | 5.855625554569653 | 4.895288376220054 | — | — | 6.855625554569658 | 4.895288376220054 |
| 2 | 6.0246317657497785 | 6.545110913930790 | 5.739735192393346 | 7.503669176369620 | 6.79723158828749 | 3.945634427684119 |
| 3 | 9.783007985803017 | 7.430381543921917 | 12.108855368234252 | 6.005900621118012 | 9.247647879782678 | 7.758266453145447 |
| 4 | 6.0246317657497785 | 7.583291925465840 | 7.843460514640638 | 7.591339840283940 | — | — |

**idealStartingState rotation:** 111.31791227546151°
**goalEndState rotation:** 173.558399900665°
**goalEndState velocity:** 0

All other fields (rotationTargets, constraintZones, pointTowardsZones, eventMarkers, globalConstraints, reversed, folder, useDefaultConstraints) are identical to `Elliot is Special.path`.

---

## Code Changes

### `AutomaticCommands.java`

Add one new public static method and one new import (`PathPlannerPath`):

```java
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * D-Pad Up: pathfinds to the start of the 254 super pass route, then follows it.
 * Picks Elliot is Special (bottom half) or Upward Elliot is Special (top half),
 * then applies PathPlanner's alliance flip for red.
 */
public static Command superPassCommand(Drive drive) {
    return Commands.defer(
        () -> {
            boolean isTopHalf = drive.getPose().getY() > FieldFlipUtil.FIELD_WIDTH_METERS / 2.0;
            boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
            String pathName = isTopHalf ? "Upward Elliot is Special" : "Elliot is Special";
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            if (isRed) path = path.flipPath();
            Logger.recordOutput("AutomaticCommands/superPass/pathName", pathName);
            Logger.recordOutput("AutomaticCommands/superPass/isRed", isRed);
            return AutoBuilder.pathfindThenFollowPath(path, CONSTRAINTS);
        },
        Set.of(drive));
}
```

### `RobotContainer.java`

Add after the existing D-Pad bindings:

```java
controller.povUp().whileTrue(AutomaticCommands.superPassCommand(drive));
```

---

## Flip Logic Summary

| Alliance | Robot Y | Path file loaded | flipPath() called |
|----------|---------|-----------------|-------------------|
| Blue | bottom (< 4.105) | Elliot is Special | No |
| Blue | top (≥ 4.105) | Upward Elliot is Special | No |
| Red | bottom | Elliot is Special | Yes |
| Red | top | Upward Elliot is Special | Yes |

---

## What is NOT in scope

- No superstructure state changes during the path — driver uses existing shoot/pass triggers independently.
- No driver override signal — releasing D-Pad Up cancels via `whileTrue` semantics.
- No new constants file — path name strings are inline in `AutomaticCommands`.
