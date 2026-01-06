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

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "BR";
  public static String camera1Name = "Reef_R";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  public static Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(
              Inches.of(-12.306612), Inches.of(-12.743715), Inches.of(7.598246 + 1.875)),
          new Rotation3d(
              Degrees.of(0).in(Radians),
              Degrees.of(-10).in(Radians),
              Degrees.of(180 + 45).in(Radians)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(Inches.of(12.615), Inches.of(-10.841124), Inches.of(7.612962 + 1.4785)),
          new Rotation3d(
              Degrees.of(0).in(Radians), Degrees.of(-10).in(Radians), Degrees.of(20).in(Radians)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };
}
