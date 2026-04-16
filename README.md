# FRC Team 6238 — 2026 Robot

WPILib Command-based Java robot with [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) logging and replay.

---

## Table of Contents

- [Subsystem Structure](#subsystem-structure)
- [Superstructure State Machine](#superstructure-state-machine)
- [Autonomous Routines](#autonomous-routines)
- [Vision & Filtering](#vision--filtering)
- [Special Features](#special-features)
- [Build & Deploy](#build--deploy)

---

## Subsystem Structure

All subsystems follow the AdvantageKit IO layer pattern:

- `FooIO` — interface with a nested `@AutoLog FooIOInputs` class and default no-op methods
- `FooIOTalonFX` / `FooIOSim` — concrete hardware/simulation implementations
- `Foo` (subsystem) — holds `FooIOInputsAutoLogged`, calls `io.updateInputs(inputs)` in `periodic()`, then `Logger.processInputs()`

### Drive

4-module swerve using **CTRE TalonFX (KrakenX60)** and **Phoenix 6**.

- 8x TalonFX motors (1 drive + 1 steer per module), CANcoder per module
- **Pigeon 2** IMU for heading
- High-frequency odometry via `PhoenixOdometryThread` at 250 Hz (CANivore) / 100 Hz (standard CAN)
- Simulation via **MapleSim** (`MapleSimSwerve`) with field obstacle physics
- PathPlanner `AutoBuilder` configured with holonomic drive controller
- Exposes `align(APTarget, BooleanSupplier)` for teleop AutoPilot assistance

### Vision

Multi-camera AprilTag pose estimation (see [Vision & Filtering](#vision--filtering)).

- 2x PhotonVision cameras: `FRONT` and `SIDE`
- Fused into drivetrain odometry via `drive::addVisionMeasurement()`

### Shooter

Flywheel + feeder for game piece ejection.

| Motor | CAN ID | Role |
|-------|--------|------|
| TalonFX | 39 | Main flywheel |
| TalonFX | 48 | Flywheel follower |
| TalonFX | 40 | Feeder |

- PID + FF flywheel velocity control
- `flywheelUpToSpeed()` checks within `0.5 RPS` tolerance
- Ball exit detected via 4% velocity drop (`FLYWHEEL_BANG_THROUGH_THRESHOLD`)
- **Shot map**: distance → flywheel RPS (2.32 m → 43.9 RPS through 4.14 m → 55.9 RPS, extrapolated to 8 m)
- **Hood angle**: fixed at 60.5°

### Hopper

Dual indexer feeding the shooter.

| Motor | CAN ID | Role |
|-------|--------|------|
| TalonFX | 41 | Main indexer |
| TalonFX | 46 | Top indexer |

- Jam detection: stall at < 2 RPS triggers reversal
- Backs up briefly before first spinup to seat game pieces

### Intake

Pivot arm + roller with jam detection and crawl-up sequence.

| Motor | CAN ID | Role |
|-------|--------|------|
| TalonFX | 54 | Pivot |
| TalonFX | 59 | Roller |
| TalonFX | 55 | Roller follower |
| CANcoder | 56 | Pivot position feedback |

- **Jam detection**: stall current > 35 A at < 8 RPS for 250 ms → reverse at −12 V for 187.5 ms, then resume
- **Crawl-up sequence**: gravity-compensated slow voltage ramp into hard stop for backlash preload
- **Pivot oscillation during shooting**: asymmetric sine wave (amplitude 10°, frequency 4 Hz, duty cycle 0.7), sweeps center upward at 15°/sec — all tunable via NetworkTables

### Superstructure

See [Superstructure State Machine](#superstructure-state-machine).

---

## Superstructure State Machine

`Superstructure.java` is the central coordinator. It uses a two-level state machine.

### Wanted States (driver input)

| State | Trigger | Description |
|-------|---------|-------------|
| `IDLE` | Release | All stopped |
| `INTAKING` | Left trigger | Intake arm down, roller spinning |
| `SHOOTING` | Right trigger | Spin up + fire at hub |
| `PASSING` | Right bumper | Spin up + pass to ally |
| `SHOOT_INTAKE` | Right trigger + intake | Simultaneous shot + intake during spin-up |
| `PASS_INTAKE` | Right bumper + intake | Simultaneous pass + intake during spin-up |
| `MANUAL_SHOOTING` | D-Pad Left | Shooting without AutoPilot heading lock |

### Current States (actual operation)

```
IDLE → SPINNING_UP → SHOOTING
                   → PASSING
     → INTAKING
```

### Key Behaviors

**Spin-up & shot sequence:**
1. Hopper back-reverses to seat game pieces (first spinup only)
2. Flywheel ramps to `ShotPlanner`-computed speed for current distance
3. Once `readyToShoot()` (speed within 0.5 RPS + hub heading within 4°): pivot oscillates, hopper indexes
4. `ballExitedFlywheel()` triggers on 4% velocity drop — confirms piece left

**No-shot recovery:**
- If no ball exits for 1.0 sec AND 1.5 sec cooldown has elapsed → drops intake, restarts spin-up from beginning

**Post-shot hold:**
- Flywheel holds shot speed for 0.5 sec after returning to IDLE (reduces back-EMF shock)

**Hub pre-spinup:**
- If `SPINUP_WHEN_HUB_ACTIVE` is enabled and match timing indicates hub is active, flywheel pre-spins to ~50 RPS during IDLE/INTAKING

### ShotPlanner

`ShotPlanner.java` iteratively solves lead time and flywheel speed from current robot pose and velocity. Interpolates from distance→RPS maps; falls back to polynomial extrapolation outside measured range.

---

## Autonomous Routines

All autos use **PathPlanner** for trajectory execution. Named commands are registered in `RobotContainer.configureNamedCommands()` via `AutoRoutines`.

### Auto Chooser Options

| Auto | Strategy |
|------|----------|
| Right Trench Mid Rush (double) | Right trench cycles × 2 (shoot 4.5 s each) |
| Left Trench Delay Mid → Depot | Delay, upper trench intake, shoot 4.5 s, pathfind to depot, shoot 4.5 s |
| Left Risky Trench Delay Mid → Depot | Same as above via alternate (riskier) path |
| Left Trench Mid Rush (double) | Upper trench cycles × 2 (4.5 s / 4.0 s shoots) |
| Right Trench Delay Mid | Delay, right trench intake, shoot 10 s |
| 254 Follow | Follow "254" path, intake, shoot 10 s |
| Elliot Special | Trench → mid intake × 3 with 4.5 s passes |
| Depot | Straight to depot, intake, shoot 10 s |
| No Intake | Straight path, shoot 10 s, no intake |

### Key Parameters

- **Auto delay**: tunable via NetworkTables (default 1.35 s)
- **Shoot duration**: 4–10 s depending on routine
- **PathPlanner constraints**: velocity 3.5 m/s, acceleration 8.0 m/s²

---

## Vision & Filtering

### Camera Configuration

| Camera | Name | Position (from center) | Yaw |
|--------|------|------------------------|-----|
| PhotonVision 0 | `FRONT` | 7" forward, 8.2" right, 20" up | −5.35° |
| PhotonVision 1 | `SIDE` | 14.75" forward, 11" left, 15" up | 90° |

Field layout: **2026 Rebuilt/Welded** AprilTag map.

### Rejection Filters

An observation is discarded if any of the following are true:

| Condition | Threshold |
|-----------|-----------|
| No tags detected | `tagCount == 0` |
| Single-tag ambiguity too high | `ambiguity > 0.14` |
| Robot Z (height) unrealistic | `|Z| > 0.65 m` |
| Out of field bounds X | `X < 0 or X > fieldLength` |
| Out of field bounds Y | `Y < 0 or Y > fieldWidth` |
| In X exclusion zone | `X ∈ [4.0, 5.292]` or `X ∈ [11.5, 12.5]` |

### Standard Deviation Computation

Accepted observations are weighted by distance and tag count before being fused:

```
stdDevFactor      = avgTagDistance² / tagCount
linearStdDev      = 0.07 m  × stdDevFactor × cameraFactor
angularStdDev     = 0.09 rad × stdDevFactor × cameraFactor
```

| Camera | `cameraFactor` | Notes |
|--------|---------------|-------|
| FRONT | 1.0 | Full trust |
| SIDE | 1.4 | Increased uncertainty (side-mounted) |

Accepted poses are sent to `drive::addVisionMeasurement()`. Rejected poses are logged separately for debugging.

---

## Special Features

### Defense Mode (Y Button)

Toggles runtime current limits via TalonFX reconfiguration:

| Subsystem | Normal | Defense |
|-----------|--------|---------|
| Drive supply / stator | 55 A / 102 A | 80 A / 120 A |
| Shooter, Hopper, Intake | Normal | Reduced (frees bus current for drive) |

Resets to OFF at every `teleopInit()`.

### Alignment (B & X Buttons)

**B Button — context-aware snap:**
- Near alliance wall → wall tower traversal
- In trench area → trench traversal (heading locked to 0°/180°)

**X Button — localization lineup:**
- Near walls → bump crossing at mirrored bump positions
- In trench → back intake into wall to shoot immediately after trench crossing

All targets are field-mirrored for red/blue alliance.

### Betaflight Actual Rate Curve

Joystick input is shaped using the **Betaflight Actual** model instead of a simple quadratic:

```
output(x) = (CENTER_SENS × x + (MAX_RATE − CENTER_SENS) × x^EXPO) / MAX_RATE
```

| Constant | Value | Effect |
|----------|-------|--------|
| `CENTER_SENS` | 0.3 | Fine linear control near center stick |
| `MAX_RATE` | 1.0 | Full rate scaling |
| `EXPO` | 3.0 | Cubic response at outer stick |

Applied independently to translation magnitude and rotation.

### Phoenix Odometry Thread

`PhoenixOdometryThread` runs at **250 Hz** (CANivore) or **100 Hz** (standard CAN), batching all CAN signal reads. The odometry lock is released before AdvantageKit serialization to minimize contention on the main loop.

### Robot Identity

`RobotIdentity` reads the roboRIO serial number at startup and selects either `COMPTunerConstants` (competition robot) or `PRACTICETunerConstants` (practice robot). All motor CAN IDs and tuning constants flow from the selected TunerConstants class.

### MapleSim Simulation

In `SIM` mode:
- `MapleSimSwerve` provides physics-based swerve modeling
- `RobotBumpSim` simulates field ramps/obstacles
- `IntakeSimulation::OverTheBumperIntake()` models game piece pickup
- `Superstructure::simulateShot()` computes ball trajectories from flywheel RPS → surface speed → projectile physics, logging hits/misses against the hub

---

## Build & Deploy

```bash
# Build (runs Spotless formatter automatically before compile)
./gradlew build

# Build without tests
./gradlew build -x test

# Run all tests
./gradlew test

# Deploy to robot
./gradlew deploy

# Run AdvantageKit log replay
./gradlew replayWatch

# Apply formatting manually (Google Java Format)
./gradlew spotlessApply
```

Spotless formatting is enforced on every `compileJava`. All `.java`, `.gradle`, `.json`, and `.md` files are formatted automatically.
