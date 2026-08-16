# 2026 Robot Architecture

Team 6238 — FRC 2026 (Rebuilt). WPILib command-based Java with AdvantageKit logging/replay.

The robot intakes fuel, indexes it, and shoots or passes at the hub. Four groups do the work: **runtime**, **mobility**, **game piece path**, and **match control**. Shared IO/logging sits under all of them.

Data moves **top → bottom each 20 ms loop**. Commands set *intent*; Superstructure and Drive apply it. Wanted-state changes take effect on the **next** Superstructure periodic.

---

## Component groups

| Group | What it is | Main types |
|-------|------------|------------|
| **Runtime** | Boot, wiring, mode, identity | `Robot`, `RobotContainer`, `Constants`, `RobotIdentity` |
| **Mobility** | Where the robot is and how it moves | `Drive`, `Vision` |
| **Game piece path** | Intake → hopper → shoot/pass | `Superstructure`, `ShotPlanner`, `Intake*`, `Hopper`, `Shooter` |
| **Match control** | Driver + auto intent | `DriveCommands`, `AutomaticCommands`, `AutoRoutines` |

```mermaid
flowchart TB
  subgraph Runtime["1. Runtime — once at boot, then each loop"]
    Robot["Robot.robotPeriodic"]
    Sched[CommandScheduler.run]
    Robot --> Sched
  end

  subgraph Periodics["2. Subsystem periodic — registration order"]
    Drive["Drive: odometry → pose"]
    Vision["Vision: tags → addVisionMeasurement"]
    Mechs["Shooter / Hopper / Intake: read sensors"]
    Super["Superstructure: plan shot + apply states"]
    Drive --> Vision --> Mechs --> Super
  end

  subgraph Match["3. Match control — after periodics"]
    Bind["Poll triggers / buttons"]
    Cmds["Execute commands<br/>teleop / auto / AutomaticCommands"]
    Bind --> Cmds
  end

  subgraph Outputs["4. Outputs same cycle"]
    Chassis["Drive velocity / path"]
    Motors["Mechanism setpoints<br/>already written in Superstructure periodic"]
  end

  Sched --> Drive
  Super --> Bind
  Cmds -->|wanted state next cycle| Super
  Cmds --> Chassis
  Super --> Motors
```

---

## What `periodic()` is

`periodic()` is a **callback**, not its own thread. Once per control cycle (~20 ms, `Constants.loopPeriodSecs`), WPILib calls `Robot.robotPeriodic()`, which calls `CommandScheduler.run()`. That method walks registered subsystems **one after another on the same thread** and invokes each `periodic()`, then polls buttons and runs command `execute()` — still on that thread.

Drive, Vision, Superstructure, and teleop commands do **not** run in parallel. They take turns in a fixed order every loop. This is a **cyclic executive** (like a PLC scan or Arduino `loop()`), not one OS task per subsystem:

```text
while robot is running:          ← one main thread, ~50 Hz
    Drive.periodic()
    Vision.periodic()
    Shooter/Hopper/Intake.periodic()
    Superstructure.periodic()
    poll Xbox / schedule commands
    command.execute() …          ← joystick, auto, pathfinding
    Robot match logging
    (sim) update physics
    sleep until next 20 ms tick
```

Everyone must finish quickly so the next cycle starts on time. If Superstructure hangs for 30 ms, Drive and commands are late too — there is no preemption between them.

Subsystems share memory on that thread (`drive.getPose()`, `wantedSuperState` are just fields). No locks are needed between them because they never preempt each other.

Background work **does** run on other threads, but it only **feeds data** into the main loop:

| Thread | Role |
|--------|------|
| Main robot thread | All `periodic()`, commands, most control |
| `PhoenixOdometryThread` | High-rate gyro/module samples → queue |
| Motor / CAN / Photon | Device firmware + vendor I/O |
| AdvantageKit logger | Serialize logs off the hot path where possible |

Drive’s `periodic()` drains the odometry queue. Vision’s `periodic()` reads cameras and calls `addVisionMeasurement`. Superstructure then uses that pose. Producers can run concurrently; **decisions and motor setpoints still happen in one serial pass**.

---

## Loop order (every 20 ms)

WPILib runs **all `periodic()` methods first**, then polls bindings, then executes commands. Subsystem order is construction order in `RobotContainer` (Lighting → Drive → Vision → Shooter → Hopper → Intake → Superstructure).

A background **Phoenix odometry thread** (250 Hz CANivore / 100 Hz CAN) samples gyro + modules into a queue the whole time.

```mermaid
flowchart TB
  Start["LoggedRobot loop start"] --> OdoQ["PhoenixOdometryThread queue<br/>async, high rate"]
  OdoQ --> P1

  subgraph P["CommandScheduler: subsystem periodic"]
    direction TB
    P1["1 Lighting — LEDs from last cycle state"]
    P2["2 Drive — batch CAN refresh, updateInputs,<br/>drain odometry queue → pose estimator"]
    P3["3 Vision — cameras, filter tags,<br/>addVisionMeasurement into Drive pose"]
    P4["4 Shooter / Hopper / Intake — updateInputs + log"]
    P5["5 Superstructure — ShotPlanner from pose,<br/>handleWantedState, applyStates → motors"]
    P1 --> P2 --> P3 --> P4 --> P5
  end

  P5 --> B["6 Poll Xbox / NT triggers<br/>schedule / cancel commands"]
  B --> C["7 Execute commands"]
  C --> Meta["8 Robot: match shift, hub-active, alerts"]
  Meta --> Sim["9 Sim only: MapleSim + fuel + bump"]
  Sim --> End["Logger flush → actuators hold until next loop"]
```

| Step | Group | What data moves |
|------|-------|-----------------|
| 1 | Runtime / lighting | Previous Superstructure state → CANdles |
| 2 | Mobility | Sensors → Drive pose (wheels + gyro) |
| 3 | Mobility | Cameras → filtered poses → **same** estimator |
| 4 | Game piece | Mechanism sensors logged; jam flags updated |
| 5 | Game piece | Pose + *current* wanted state → flywheel/feeder/indexer/intake |
| 6–7 | Match control | Sticks/buttons/auto → new wanted state + chassis speeds |
| 8–9 | Runtime | Match metadata; physics step in SIM |

**Lag:** step 7 may change `WantedState` or start a path; Superstructure does not see that until **step 5 next loop**. Chassis `runVelocity` from a drive command applies in step 7 this loop. Shot heading used by angle-lock drive was computed in step 5 this loop (pose already includes this loop's vision).

---

## 1. Runtime

Once: `Main` → `Robot` (`LoggedRobot`) → AdvantageKit receivers → `RobotContainer` (IO + subsystems + bindings + auto chooser).

Each loop: `robotPeriodic` → `CommandScheduler.run()` → shift / hub / battery logging. SIM also runs `updateSimulation()` after that.

```mermaid
flowchart TB
  Main --> Robot
  Robot --> Logger["AdvantageKit start"]
  Logger --> RC[RobotContainer]
  RC -->|REAL / SIM / REPLAY| IO[IO implementations]
  IO --> Subs[Subsystems registered with scheduler]
```

| Mode | When | Hardware side |
|------|------|----------------|
| `REAL` | On the roboRIO | TalonFX, Pigeon, PhotonVision |
| `SIM` | Desktop | MapleSim + `*IOSim` |
| `REPLAY` | Log replay | No-op IO; inputs from WPILOG |

`RobotIdentity` chooses COMP vs practice tuner constants from the RIO serial.

---

## 2. Mobility

One pose pipeline. Drive runs **before** Vision in the same loop, then Vision writes corrections into the estimator immediately.

```mermaid
flowchart TB
  Gyro[Gyro] --> Thread[PhoenixOdometryThread]
  Enc[Module encoders] --> Thread
  Thread -->|queued samples| DrivePer["Drive.periodic"]
  DrivePer --> Est[Pose estimator]
  Cams[Photon cameras] --> VisPer["Vision.periodic"]
  Est -->|getPose for sim cameras| VisPer
  VisPer -->|filtered addVisionMeasurement| Est
  Est --> Pose[Robot pose]
  Pose --> Later["Superstructure + drive commands later in loop"]
```

- **Drive** — 4-module Kraken swerve, Phoenix odometry, PathPlanner `AutoBuilder`. Velocity setpoints come from commands **after** periodics.
- **Vision** — `FRONT` + `SIDE`; drop bad tags (ambiguity, Z, out of field); remaining poses update Drive.

---

## 3. Game piece path

Intake, hopper, and shooter are not driven independently during a cycle. **Superstructure** runs last among subsystems: operators/autos already set a *wanted* state on a prior loop; this periodic maps it to *current* state and writes motors. `ShotPlanner` uses the pose just updated by Drive + Vision.

```mermaid
flowchart TB
  Pose["Drive pose + speeds<br/>just updated"] --> Planner[ShotPlanner]
  Wanted["Wanted state<br/>from last command cycle"] --> Super[Superstructure.periodic]
  Planner --> Super
  Super --> Handle[handleWantedState]
  Handle --> Apply[applyStates]
  Apply --> Intake
  Apply --> Hopper
  Apply --> Shooter
```

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> INTAKING: want intake
  IDLE --> SPINNING_UP: want shoot / pass
  INTAKING --> SPINNING_UP: want shoot / pass
  SPINNING_UP --> SHOOTING: flywheel + aim ready
  SPINNING_UP --> PASSING: flywheel + aim ready
  SHOOTING --> SPINNING_UP: heading lost
  PASSING --> SPINNING_UP: heading lost
  SHOOTING --> IDLE: want idle
  PASSING --> IDLE: want idle
  INTAKING --> IDLE: want idle
  IDLE --> PIT_SHOOTING: pit shoot
  PIT_SHOOTING --> IDLE: want idle
```

| Piece | Role |
|-------|------|
| Intake (pivot + roller) | Collect; jam reverse on stall |
| Hopper (indexer + top) | Feed shooter; unjam on stall |
| Shooter (flywheel + feeder) | Velocity-controlled eject |
| ShotPlanner | Distance maps + moving-target lead |

Teleop picks shoot vs pass from field X (`Constants.SHOULD_PASS`). `*_INTAKE` wanted states keep collecting while shooting/passing.

---

## 4. Match control

Runs **after** periodics. This group only expresses intent.

```mermaid
flowchart TB
  Poll[Poll triggers] --> Which{Scheduled command}
  Which --> Tele["Teleop DriveCommands<br/>sticks → chassis"]
  Which --> SuperWant["RT/RB/LB/… → setWantedSuperState"]
  Which --> Assist["AutomaticCommands<br/>pathfind / follow"]
  Which --> Auto["AutoRoutines<br/>followPath + shoot/pass"]
  Tele --> Chassis["Drive.runVelocity now"]
  Assist --> Chassis
  Auto --> Chassis
  SuperWant --> Next["Superstructure next periodic"]
  Auto --> Next
```

| Source | Intent |
|--------|--------|
| Default teleop | Joystick drive (`DriveCommands`) |
| RT / RB | Want shoot or pass + lock heading to this loop's shot setpoint |
| LB / LT / Y / A | Intake, reverse, pivot |
| B / X / D-pad | `AutomaticCommands` pathfind assist |
| Auto chooser | `AutoRoutines`: PathPlanner follow → shoot/pass → intake |

Autos are hand-built sequences around PathPlanner paths (not NamedCommands). Paths live in `src/main/deploy/pathplanner/`.

---

## Shared infrastructure

Every hardware subsystem uses the same AdvantageKit IO split so REAL, SIM, and REPLAY share one control loop. This is **steps 2–5** of each cycle:

```mermaid
flowchart TB
  HW[Sensors / motors] --> Impl["io.updateInputs"]
  Impl --> Log["Logger.processInputs"]
  Log --> Logic["Domain logic in periodic"]
  Logic --> Set["IO setters — Superstructure now,<br/>Drive commands after periodics"]
  Set --> HW
```

Tests mock the IO interface, not the subsystem.

Also in this bucket: CANdle lighting, on-robot test mode, MapleSim arena, `./gradlew replayWatch`.

---

## Related

- [`AGENTS.md`](../AGENTS.md) — commands, IO pattern, tests
- [`README.md`](../README.md) — operator summary
- [`docs/superpowers/`](superpowers/) — feature specs
