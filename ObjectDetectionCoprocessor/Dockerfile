# Use the base image that matches your Jetson's R36.4 release
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0 AS builder

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    clang-15 \
    git \
    curl \
    unzip \
    openjdk-17-jdk \
    libopenblas-dev \
    libatlas-base-dev \
    file \
    && rm -rf /var/lib/apt/lists/*

# --- INSTALL CUSTOM OPENCV ARTIFACT ---
COPY opencv_custom_arm64.tar.gz /tmp/
RUN tar -xzvf /tmp/opencv_custom_arm64.tar.gz -C /usr/local && \
    rm /tmp/opencv_custom_arm64.tar.gz

# --- INSTALL CUSTOM NVIDIA ARTIFACT ---
COPY nvidia_runtime_libs.tar.gz /tmp/
# CRITICAL: Purge any existing stubs/broken links before extracting real ones
RUN rm -rf /usr/lib/aarch64-linux-gnu/nvidia/* && \
    tar -xzvPf /tmp/nvidia_runtime_libs.tar.gz -C / && \
    rm /tmp/nvidia_runtime_libs.tar.gz

# --- REPAIR NVIDIA SYMLINKS ---
RUN for dir in /usr/lib/aarch64-linux-gnu/nvidia /lib/aarch64-linux-gnu /usr/local/cuda/lib64; do \
        if [ -d "$dir" ]; then \
            cd "$dir" && for f in *.so.*; do \
                base=$(echo $f | sed 's/\.so\..*$/.so/'); \
                if [ ! -f "$base" ] && [ ! -L "$base" ]; then ln -s "$f" "$base"; fi; \
            done; \
        fi; \
    done

# --- CONFIGURE LINKER ---
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/lib/aarch64-linux-gnu/nvidia" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/lib/aarch64-linux-gnu" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/local/cuda/targets/aarch64-linux/lib" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/local/cuda/lib64" >> /etc/ld.so.conf.d/frc_robot.conf && \
    ldconfig

# Set up Gradle
ENV GRADLE_VERSION=8.5
RUN curl -L https://services.gradle.org/distributions/gradle-${GRADLE_VERSION}-bin.zip -o gradle.zip \
    && unzip gradle.zip -d /opt \
    && rm gradle.zip
ENV PATH="/opt/gradle-${GRADLE_VERSION}/bin:${PATH}"

WORKDIR /app
COPY . .

# Build the project
RUN ./gradlew build --no-daemon

# Final stage
FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# --- INSTALL RUNTIME MATH LIBS ---
# These are small and handle their own symlinks correctly
RUN apt-get update && apt-get install -y \
    libopenblas0-pthread \
    libatlas3-base \
    libtbb12 \
    libatomic1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy the built binary and internal libraries from builder
COPY --from=builder /app/build-gradle/cmake/inference_app /app/
COPY --from=builder /app/build-gradle/cmake/lib*.so* /app/
COPY --from=builder /app/data /app/data
COPY --from=builder /app/reefscape.engine /app/
COPY --from=builder /app/data/reefscape-labels.txt /app/data/

# 2. Extract both "Golden" tarballs directly into the final stage
COPY opencv_custom_arm64.tar.gz /tmp/
COPY nvidia_runtime_libs.tar.gz /tmp/
RUN tar -xzvf /tmp/opencv_custom_arm64.tar.gz -C /usr/local && \
    tar -xzvPf /tmp/nvidia_runtime_libs.tar.gz -C / && \
    rm /tmp/*.tar.gz

# 3. Setup runtime linker
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/lib/aarch64-linux-gnu/nvidia" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/lib/aarch64-linux-gnu" >> /etc/ld.so.conf.d/frc_robot.conf && \
    echo "/usr/local/cuda/lib64" >> /etc/ld.so.conf.d/frc_robot.conf && \
    ldconfig

# Default command with mandatory arguments
ENTRYPOINT ["/app/inference_app"]
CMD ["/app/reefscape.engine", "/app/data/reefscape-labels.txt"]
