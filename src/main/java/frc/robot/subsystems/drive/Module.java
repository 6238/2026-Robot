// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private final String logKey;
  private static final int MAX_ODOMETRY_SAMPLES = 20;
  // Pre-allocated pool — elements are mutated in-place each cycle, no per-cycle allocation.
  private final SwerveModulePosition[] odometryPositions;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    this.logKey = "Drive/Module" + index;
    odometryPositions = new SwerveModulePosition[MAX_ODOMETRY_SAMPLES];
    for (int i = 0; i < MAX_ODOMETRY_SAMPLES; i++) {
      odometryPositions[i] = new SwerveModulePosition();
    }
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  /** Called under odometryLock — reads hardware into inputs struct. No logging. */
  public void updateHardwareInputs() {
    io.updateInputs(inputs);
  }

  /** Called outside odometryLock — serializes inputs to AK log and updates alerts. */
  public void logAndProcessInputs() {
    Logger.processInputs(logKey, inputs);

    // Calculate positions for odometry — mutate pool in-place, no per-cycle allocation
    int sampleCount = inputs.odometryTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i].distanceMeters =
          inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      odometryPositions[i].angle = inputs.odometryTurnPositions[i];
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Convenience delegate — preserves existing call sites in tests. */
  public void periodic() {
    updateHardwareInputs();
    logAndProcessInputs();
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    runSetpoint(state, 0.0);
  }

  /**
   * Runs the module with the specified setpoint state and torque-current feedforward (amps).
   * Mutates the state to optimize it.
   */
  public void runSetpoint(SwerveModuleState state, double torqueCurrentFF) {
    // Note: no optimize() here. The SwerveSetpointGenerator already handles direction
    // optimization by rate-limiting steer velocity. Calling optimize() after the generator
    // corrupts previousSetpoint's model of module angles and causes 180° flip oscillation.
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints. When stopped, use open-loop 0 instead of velocity PID at 0 to avoid
    // active stall current fighting brake mode.
    if (Math.abs(state.speedMetersPerSecond) < 0.01) {
      io.setDriveOpenLoop(0.0);
    } else {
      io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius, torqueCurrentFF);
    }
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  /** Runs the steer motor open-loop for SysId characterization. Stops drive explicitly. */
  public void runSteerCharacterization(double output) {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(output);
  }

  // public void runCharacterization(double output) {
  //   io.setDriveOpenLoop(0.0);
  //   io.setTurnOpenLoop(output);
  // }

  // /** Characterize robot angular motion. */
  // public void runCharacterization(double output) {
  //     io.setDriveOpenLoop(output);
  //     io.setTurnPosition(new Rotation2d(constants.LocationX,
  // constants.LocationY).plus(Rotation2d.kCCW_Pi_2));
  // }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }

  /** Returns the drive motor applied voltage. Used by SysId log consumer. */
  public double getDriveAppliedVolts() {
    return inputs.driveAppliedVolts;
  }

  /** Returns the drive motor position in radians. Used by SysId log consumer. */
  public double getDrivePositionRad() {
    return inputs.drivePositionRad;
  }

  /** Returns the turn motor applied voltage. Used by SysId log consumer. */
  public double getTurnAppliedVolts() {
    return inputs.turnAppliedVolts;
  }

  /** Returns the drive motor velocity in rad/s. Used by test mode. */
  public double getDriveVelocityRadPerSec() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the steer motor velocity in rad/s. Used by test mode. */
  public double getSteerVelocityRadPerSec() {
    return inputs.turnVelocityRadPerSec;
  }

  /** Runs the steer motor open-loop. Used by test mode. */
  public void runSteerOpenLoop(double output) {
    io.setTurnOpenLoop(output);
  }

  public double getDriveSupplyCurrentAmps() {
    return inputs.driveSupplyCurrentAmps;
  }

  public double getTurnSupplyCurrentAmps() {
    return inputs.turnSupplyCurrentAmps;
  }

  /** Delegates to {@link ModuleIO#setDriveCurrentLimits} for runtime current limit changes. */
  public void setDriveCurrentLimits(double supplyAmps, double statorAmps) {
    io.setDriveCurrentLimits(supplyAmps, statorAmps);
  }
}
