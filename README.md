# 2026 Robot Code

## Installation

To work on the Robot Codebase simply:
```
git clone https://github.com/6238/2026-Robot
```

After cloning the robot code, if you want to work on the Jetson Object Detection code run:
```
git lfs install
git lfs pull
```

## Deploying to the Robot

You can use <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>P</kbd> (Windows) or <kbd>Cmd</kbd> + <kbd>Shift</kbd> + <kbd>p</kbd> (Mac) to open the command window. Then Select ```WPILIB: Deploy Robot Code```.

Alternativly you can run ```./gradlew deploy```.

## Deploying the Jetson Code
Refer to [SETUP_JETSON](https://github.com/6238/2026-Robot/blob/main/SETUP_JETSON.md) for information on building and deploying the docker container on the Jetson Nano.

## Object Detection Structure
<img width="684" height="718" alt="image" src="https://github.com/user-attachments/assets/3e98542c-b3cd-4438-92c9-e263e1111bc7" />
