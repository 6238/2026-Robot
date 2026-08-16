# Agent Setup Guide

Instructions for a cold agent (or developer) setting up a build environment for this repository on a **generic machine**. Do not assume WPILib VS Code, a previous session, or team-specific tooling is already configured.

## What this repo is

FRC Team 6238 **2026** robot code:

- Language: **Java 17**
- Framework: WPILib Command-based (`edu.wpi.first.GradleRIO` **2026.x**)
- Build: Gradle wrapper (`gradlew` / `gradlew.bat`)
- Logging: AdvantageKit
- Tests: JUnit 5 + Mockito

Team number: **6238** (see `.wpilib/wpilib_preferences.json`).

## Goal

Be able to run:

```bash
./gradlew build
```

(or `.\gradlew.bat build` on Windows) and get **BUILD SUCCESSFUL**.

## Prerequisites checklist

Verify each item before changing code.

| Requirement | Notes |
|-------------|--------|
| Git | Clone/fetch only; do not rewrite history unless asked |
| JDK **17** | Must match `sourceCompatibility` / `targetCompatibility` in `build.gradle` |
| Network | First build downloads WPILib + vendordeps from Maven |
| Disk | Gradle cache + native libs can be large (~1+ GB on first build) |

Optional but useful:

| Tool | Why |
|------|-----|
| WPILib 2026 install | Ships a known-good Temurin JDK 17 and vendor tooling |
| `gh` | GitHub PRs/issues (only if the user asks for GitHub work) |

## Step 1 — Locate or install JDK 17

### Prefer an existing JDK

Search common locations before installing:

**Windows**

- `C:\Users\Public\wpilib\2026\jdk` (WPILib installer default)
- `C:\Program Files\Eclipse Adoptium\jdk-17*`
- `C:\Program Files\Microsoft\jdk-17*`
- `C:\Program Files\Java\jdk-17*`

**macOS**

- `/Users/Shared/wpilib/2026/jdk`
- `/Library/Java/JavaVirtualMachines/*/Contents/Home`
- Homebrew: `/opt/homebrew/opt/openjdk@17` or `/usr/local/opt/openjdk@17`

**Linux**

- `/usr/lib/jvm/java-17-openjdk*`
- `/usr/lib/jvm/temurin-17*`
- `~/wpilib/2026/jdk`

Confirm:

```bash
java -version
# Expect: openjdk / java version "17.x"
javac -version
```

### If no JDK 17 is present

Install **Temurin 17** (or any OpenJDK 17) via the OS package manager, Adoptium, or the [WPILib 2026 installer](https://docs.wpilib.org/). Do **not** use Java 21+ as the project compiler target unless `build.gradle` is updated.

## Step 2 — Set `JAVA_HOME` for this shell

Gradle wrapper fails with exit code `9009` / “JAVA_HOME is not set” if neither `JAVA_HOME` nor `java` on `PATH` is available.

**Windows (PowerShell)** — example with WPILib JDK:

```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
java -version
```

**macOS / Linux**:

```bash
export JAVA_HOME="/path/to/jdk-17"
export PATH="$JAVA_HOME/bin:$PATH"
java -version
```

### Persistence (agent policy)

- **Do not** commit machine-specific JDK paths into the shared repo (`.vscode/settings.json`, etc.).
- For interactive Cursor/VS Code users, prefer **user-level** editor settings, not workspace settings checked into git.
- For agent shells, set `JAVA_HOME` in the current session (or document the path you discovered). Agent shells often **do not** inherit IDE terminal env vars.

## Step 3 — First build

From the repository root:

**Windows**

```powershell
.\gradlew.bat build
```

**macOS / Linux**

```bash
./gradlew build
```

Faster compile-only check:

```bash
./gradlew build -x test
```

### What a healthy first build does

1. Downloads Gradle (per `gradle/wrapper/gradle-wrapper.properties`)
2. Resolves WPILib + vendordeps (`vendordeps/*.json`)
3. Runs Spotless (`spotlessApply` before `compileJava`)
4. Regenerates `src/main/java/frc/robot/BuildConstants.java` (gversion)
5. Compiles main + test sources, runs tests (unless `-x test`)

First builds can take many minutes on a cold cache. Subsequent builds are much faster.

## Step 4 — Verify

| Check | Expected |
|-------|----------|
| `java -version` | 17.x |
| `./gradlew build -x test` | `BUILD SUCCESSFUL` |
| `./gradlew test` | Tests pass (or known failures called out) |
| Spotless | Auto-runs on compile; manual: `./gradlew spotlessApply` |

Useful targeted tests:

```bash
./gradlew test --tests "subsystems.ShooterTest"
./gradlew test --tests "subsystems.ShooterTest.setFlywheelRPM_callsSetFlywheelSpeed"
```

## Agent working rules for this repo

1. **Prefer the Gradle wrapper** — never require a globally installed Gradle.
2. **Use the WPILib JDK when present** — it matches the season toolchain.
3. **Leave `BuildConstants.java` alone** — it is auto-generated on every build; do not include noisy regenerations in commits unless the user explicitly wants them.
4. **Do not commit editor/JDK path personalization** — keep shared `.vscode` free of machine-local `JAVA_HOME` / `java.jdt.ls.java.home` paths.
5. **Formatting is mandatory** — Spotless (Google Java Format) runs before compile; fix format failures with `./gradlew spotlessApply`.
6. **Tests matter** — `./gradlew build` runs tests. If only compile is needed, use `-x test` and say so.
7. **Deploy is optional** — `./gradlew deploy` needs a roboRIO on the network; skip unless asked.
8. **Read `CLAUDE.md` and `README.md`** for architecture, subsystems, and test patterns after the environment works.

## Common failures

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `JAVA_HOME is not set` / `java` not found | No JDK on PATH | Install JDK 17; export `JAVA_HOME` |
| Wrong Java major version | JDK 11/21 selected | Point `JAVA_HOME` at JDK 17 |
| Dependency download errors | Offline / firewall | Restore network; retry build |
| Spotless check fails | Unformatted edits | `./gradlew spotlessApply` |
| Unit test failures after logic changes | Stale tests | Update tests to match current behavior; do not skip without saying so |
| HAL / Alert NPEs in tests | Missing HAL init or `*Connected=false` | Follow test patterns in `CLAUDE.md` (`HAL.initialize`, mock `connected=true`) |
| Agent shell missing IDE env | Cursor terminal env not inherited | Set `JAVA_HOME` explicitly in the agent command |

## Minimal “cold start” script

**Windows PowerShell**

```powershell
$candidates = @(
  "C:\Users\Public\wpilib\2026\jdk",
  "$env:ProgramFiles\Eclipse Adoptium",
  "$env:ProgramFiles\Microsoft"
)
# Prefer WPILib JDK if present; otherwise require java already on PATH
if (Test-Path "C:\Users\Public\wpilib\2026\jdk\bin\java.exe") {
  $env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
  $env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
}
java -version
.\gradlew.bat build -x test
```

**macOS / Linux bash**

```bash
if [ -d "/Users/Shared/wpilib/2026/jdk" ]; then
  export JAVA_HOME="/Users/Shared/wpilib/2026/jdk"
elif [ -d "$HOME/wpilib/2026/jdk" ]; then
  export JAVA_HOME="$HOME/wpilib/2026/jdk"
fi
if [ -n "${JAVA_HOME:-}" ]; then
  export PATH="$JAVA_HOME/bin:$PATH"
fi
java -version
./gradlew build -x test
```

## Done when

- JDK 17 is available via `JAVA_HOME` / `PATH`
- `./gradlew build` (or `build -x test`) succeeds on this machine
- No machine-specific JDK paths were committed to the shared repo
