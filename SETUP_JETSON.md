# Jetson Coprocessor Setup Guide

This guide describes how to set up your laptop to build and deploy code to the robot's Jetson coprocessor. This only needs to be done **once** per student laptop.

## Prerequisites
- **Docker Desktop**: Must be installed and running on your machine.
- **SSH Client**: Built into macOS and Windows 11.
- **Git & Git LFS**: To clone the repository and download the large library bundles.
  - macOS: `brew install git-lfs`
  - Windows: Download from [git-lfs.github.com](https://git-lfs.github.com/)
  - Once installed, run: `git lfs install`

---

## Step 1: Generate your SSH Key
Open your terminal and run the following command to create a unique security key for your laptop:

```bash
ssh-keygen -t ed25519
```
*Press **Enter** for all prompts (do not set a passphrase).*

## Step 2: Authorize your Laptop
Copy your new key to the Jetson so you can log in without a password. 
**Note:** You will need the password for the `frc6238` user for this step.

```bash
ssh-copy-id frc6238@10.62.38.200 (or whatever ip of Jetson is)
```

## Step 3: Configure Docker Context
Tell your local Docker command to send its build instructions to the Jetson instead of your own laptop.

```bash
# Create the link to the Jetson
docker context create jetson --docker "host=ssh://frc6238@10.62.38.200"  (or whatever IP of Jetson is)

# Switch your machine to "Jetson Mode"
docker context use jetson
```

---

## 4. Get the Code
The vision code is stored as a "submodule" inside the main robot repository. To download everything correctly, you must clone with the `--recursive` flag.

### For a New Clone:
```bash
git clone --recursive https://github.com/6238/ObjectDetectionRobot.git
```

### If you already cloned (and the vision folder is empty):
```bash
cd ObjectDetectionRobot
git submodule update --init --recursive
```

---

## How to Build and Deploy
Once setup is complete, you can build the vision code and deploy it to the Jetson using a single command from the project root:

```bash
./gradlew deploy-jetson
```

## Troubleshooting
- **Verify LFS**: After cloning, check the size of `ObjectDetectionCoprocessor/nvidia_runtime_libs.tar.gz`. If it is only 100 bytes, you forgot to install **Git LFS** (see Prerequisites). Run `git lfs pull` to fix it.
- **Verify Connection**: Run `docker ps`. You should see the containers currently running on the Jetson. If you see an error, check your network connection to the robot.
- **Switch back to Local**: If you need to run Docker containers on your own laptop again, run:
  ```bash
  docker context use default
  ```

