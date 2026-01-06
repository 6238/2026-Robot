# Jetson Vision Coprocessor: Architecture & Build Guide

This document explains the architecture, git structure, and Docker-based deployment strategy for the FRC 6238 Jetson vision system.

## 1. Repository Structure
The vision code is maintained as a separate repository to keep development isolated from the main robot code.

- **Git Submodule**: The `ObjectDetectionCoprocessor` directory is a git submodule.
- **Handling Submodules**: When cloning the robot code for the first time, you must run:
  ```bash
  git submodule update --init --recursive
  ```
- **Why?**: This ensures that different teams can work on vision and robot code independently while keeping a specific "known-good" version of the vision code linked to the main robot project.

## 2. Remote Build Architecture (Docker Contexts)
We use **Remote Docker Contexts** to build the image.
- **The Workflow**: You run `./gradlew deploy-jetson` on your laptop. Gradle tells your local Docker client to send the "build context" (source code and libraries) to the Jetson over SSH.
- **Why?**: 
  - **Native Performance**: Building directly on the Jetson's ARM64 processor is faster and more reliable than emulating ARM64 on an Intel/Apple Silicon Mac.
  - **Zero Setup**: Students don't need to install CUDA, TensorRT, or OpenCV on their laptops. Only Docker is required.

## 3. The "Tarball Injection" Strategy
To ensure the Docker container matches the physical Jetson hardware perfectly, we use "Tarball Injection."

- **The Problem**: NVIDIA's L4T (Linux for Tegra) drivers and certain libraries (OpenCV 4.10.0, CUDA stubs) are difficult to install via standard `apt-get` inside a container because they require hardware-level access during installation.
- **The Solution**: We "bundle" the known-good libraries from a working Jetson host into compressed files:
  1. `opencv_custom_arm64.tar.gz`: The custom-built OpenCV 4.10.0 with CUDA support.
  2. `nvidia_runtime_libs.tar.gz`: The 1.2GB "Golden" bundle of NVIDIA drivers, TensorRT, and system math libraries.
- **Why?**: This guarantees that the version of OpenCV and CUDA running in the container is **bit-for-bit identical** to the version running on the host Jetson.

## 4. Building and Deploying
The main robot `build.gradle` includes several lifecycle tasks. The most common workflow is:

```bash
# Build the code and start it in the background on the Jetson
./gradlew deploy-jetson start-jetson
```

- **`deploy-jetson`**: Triggers a `docker build` using BuildKit. It uses Clang-15 and the injected libraries to compile the `inference_app`.
- **`start-jetson`**: Restarts the vision system with the latest code (see Lifecycle Management below).

### ⚠️ Critical Note on Updating
Running `deploy-jetson` alone will **not** update the robot's active vision code. 
- `deploy-jetson` builds the new software, but the old container will continue running the old code.
- You **must** run `start-jetson` to kill the old container and launch the new one.
- **The Golden Command**: Always use `./gradlew deploy-jetson start-jetson` to ensure the robot is actually running your latest changes.

## 5. Lifecycle Management (Robot Mode)
To ensure the vision system is reliable during a match, we run it as a **managed background service**.

### Automatic Startup and Recovery
The container is started with the following flags:
- `--restart always`: The vision code starts automatically when the Jetson boots and restarts immediately if the application crashes.
- `-d` (Detached): Runs in the background so it doesn't close when you disconnect your laptop.
- `-t` (TTY): Simulates a real terminal, ensuring logs are flushed immediately and hardware drivers initialize correctly.

### Monitoring the Robot
Since the code runs in the background, you can monitor it using Docker's logging system:
```bash
# View live logs (FPS, NetworkTables status, detections)
docker logs -f jetson-vision
```

## 6. Runtime Configuration (The "Why" of the Run Command)
The `start-jetson` task uses these critical flags to enable the Jetson's hardware:
- `--runtime nvidia`: **REQUIRED**. Allows the container to access the Jetson's GPU.
- `--shm-size=1g`: Increases Shared Memory. Needed for TensorRT to load large engine files.
- `--ulimit memlock=-1`: Allows the GPU drivers to pin memory for high-speed transfers.
- `--device /dev/video0:/dev/video0`: Maps the physical camera into the container.

## 7. Summary of Key Files
- `Dockerfile`: Defines the two-stage build (Builder stage with compilers -> Runtime stage with only the binary and libraries).
- `CMakeLists.txt`: Configured with `-rpath-link` to force the linker to find the injected libraries during compilation.
- `reefscape.engine`: The optimized TensorRT model file.

## 8. Internet & Connectivity Requirements

### The First Build (Internet Required)
The very first time you run `./gradlew deploy-jetson`, the Jetson **must** have a high-speed internet connection to pull the 4GB+ base image and system packages.

### Subsequent Builds (Offline Capable)
Once the first build is successful, everything is cached.
- **No Internet Needed**: Rebuilding and redeploying code at a competition works 100% offline.
- **Radio Connectivity**: Your laptop only needs to be on the same local network (Robot Radio) as the Jetson.

| Phase | Internet Needed? | Reason |
|-------|------------------|--------|
| **Initial Setup** | **YES** | Pulling base image & apt packages |
| **Code Changes** | **NO** | Re-using cached layers & local source |
| **Competition Deployment** | **NO** | Local SSH transfer to Jetson |
