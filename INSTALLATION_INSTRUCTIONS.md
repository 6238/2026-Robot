# Installation instructions

How to set up your computer and build the Team **6238** 2026 robot code.

You need:

- A computer on Windows 10/11 (64-bit), Ubuntu 22.04+, or macOS 13.3+
- Internet for the first build (libraries download automatically)
- This repository cloned to your machine

---

## Recommended: install WPILib 2026

This is what most people on the team should do. It installs Java, VS Code with the WPILib tools, and the dashboards.

1. Open the official guide: [Installing WPILib](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
2. Download and run the **2026** installer
3. Choose a full install if you are not sure which option to pick
4. Open this project in the WPILib copy of VS Code

After install, Java is usually here:

- Windows: `C:\Users\Public\wpilib\2026\jdk`
- Mac / Linux: `~/wpilib/2026/jdk`

---

## Alternative: Java only (no WPILib VS Code)

Use this if you already have an editor you like and only need to compile from a terminal.

1. Install Java **17** from [Adoptium Temurin](https://adoptium.net/temurin/releases/?version=17) (get the JDK, not just a JRE)
2. Set `JAVA_HOME` to that install folder, and add `bin` inside it to your `PATH`
3. Open a terminal in the project folder and run the build commands below

---

## Build the code

Open a terminal in the project root (the folder that contains `build.gradle`).

**Windows**

```bat
.\gradlew.bat build
```

**Mac / Linux**

```bash
./gradlew build
```

The first run can take a while. Gradle downloads WPILib and vendor libraries (motor controllers, vision, PathPlanner, and so on) for you. Later builds are much faster.

### Common commands

| What you want | Windows | Mac / Linux |
|---------------|---------|-------------|
| Build and run tests | `.\gradlew.bat build` | `./gradlew build` |
| Build without tests | `.\gradlew.bat build -x test` | `./gradlew build -x test` |
| Run tests only | `.\gradlew.bat test` | `./gradlew test` |
| Put code on the robot | `.\gradlew.bat deploy` | `./gradlew deploy` |
| Auto-format the code | `.\gradlew.bat spotlessApply` | `./gradlew spotlessApply` |

### From WPILib VS Code

1. Open the Command Palette (`Ctrl+Shift+P` on Windows/Linux, `Cmd+Shift+P` on Mac)
2. Run **Build Robot Code** or **Deploy Robot Code**

More detail: [Building and Deploying Robot Code](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/deploying-robot-code.html).

**Important:** Do not turn the robot off while code is deploying. That can damage the roboRIO’s filesystem and force a re-image.

---

## Libraries already included

Third-party libraries are listed in the `vendordeps/` folder. Versions are fixed there, so everyone on the team gets the same Phoenix, AdvantageKit, PathPlanner, and PhotonVision builds.

You do not need to install those by hand for a normal setup. Building the project downloads them.

If you later need a new vendor library, use **WPILib: Manage Vendor Libraries** in VS Code, or ask someone who has done it before. Guide: [3rd Party Libraries](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html).

---

## Install Phoenix Tuner X (motor / PID tuning)

Our drive and mechanisms use **CTRE Phoenix 6** (Talon FX, CANcoder, Pigeon 2). **Phoenix Tuner X** is the app you use to see devices on the CAN bus, update firmware, plot signals, and tune PID (and related) gains.

### Install the app

1. Install **Phoenix Tuner X** from your platform’s store, or from CTR’s software page:
   - Docs: [Phoenix Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html)
   - Downloads / offline installer: [CTR Electronics software](https://store.ctr-electronics.com/pages/software)
2. On Windows you can use the **Microsoft Store** listing or the **Phoenix Offline Installer** (includes Tuner X)
3. Mac / iPhone / Android: use the App Store or Google Play listing linked from the docs above

Supported roughly: Windows 11 (and recent Windows 10), macOS 14+, iOS 15+, Android 9+ — see the docs for the current list.

### Connect and use it

1. Power on the robot (roboRIO + CAN devices)
2. Connect your laptop the way you usually do for deploy (USB to the roboRIO is common; USB-to-CAN / CANivore also works when set up that way)
3. Open **Phoenix Tuner X** and connect to the robot
4. Pick a device (for example a Talon FX) to:
   - Run a self-test / check firmware
   - Plot position, velocity, voltage, current
   - Edit control / PID configs and apply them to the device

Official walkthrough: [Phoenix Tuner X documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html).

**Note:** Gains you change only in Tuner live on the device until code overwrites them on enable/boot. Permanent team defaults still belong in robot code (for example TunerConstants / subsystem configs) so everyone gets the same values after deploy.

### Related: WPILib SysId

For full system identification (suggesting feedforward/PID from characterization runs), WPILib’s **SysId** tool is installed with WPILib. Open it from the WPILib tools menu after the WPILib install. That is separate from Tuner X; Tuner X is what you use day-to-day on CTRE hardware.

---

## Optional extras

### Keep the local Gradle cache for 30 days

Only needed if you care about the project’s build-cache cleanup setting.

1. Copy `gradle/build-cache-retention.init.gradle` into:
   - Windows: `%USERPROFILE%\.gradle\init.d\`
   - Mac / Linux: `~/.gradle/init.d/`
2. Or skip this — builds still work with Gradle’s defaults

### Block unformatted code on git push

Code is formatted automatically when you compile. To also stop a push if formatting was skipped:

```bash
git config core.hooksPath .githooks
```

Team conventions (branch names, formatting, tests) are in [`STYLEGUIDE.md`](STYLEGUIDE.md).

---

## Where to go next

- [`README.md`](README.md) — what the robot software does
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — how the major pieces fit together
- [`STYLEGUIDE.md`](STYLEGUIDE.md) — how we name branches and format code
- [WPILib documentation](https://docs.wpilib.org/en/stable/)
- [AdvantageKit documentation](https://docs.advantagekit.org/)
- [Phoenix Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html) — CTRE device config and PID tuning

If `build` fails, check that Java 17 is installed, `JAVA_HOME` is set (if you are not using WPILib VS Code), and you have network access for the first download. Then ask on the team software channel with the error text from the terminal.
