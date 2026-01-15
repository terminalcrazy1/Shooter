// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  // Field dimensions (meters)
  public static final double fieldLength = AprilTagLayoutType.WELDED.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.WELDED.getLayout().getFieldWidth();

  // AprilTag info
  public static final int aprilTagCount = AprilTagLayoutType.WELDED.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.WELDED;

  // Scoring element positions
  // calculated from feild layout drawings147.47
  public static final Pose2d allianceHubPosition =
      new Pose2d(
          Units.inchesToMeters(181.56),
          FieldConstants.fieldWidth / 2.0,
          Rotation2d.fromDegrees(180));
  public static final Pose2d allianceClimbPosition =
      new Pose2d(
          Units.inchesToMeters(41.56), Units.inchesToMeters(147.47), Rotation2d.fromDegrees(0));

  @Getter
  private enum AprilTagLayoutType {
    WELDED("apriltags/2026-rebuilt-welded.json");
    private final AprilTagFieldLayout layout;
    private final String layoutString;

    AprilTagLayoutType(String name) {
      try {
        // Load JSON from deploy folder
        Path jsonPath = Path.of(Filesystem.getDeployDirectory().getPath(), name);

        layout = new AprilTagFieldLayout(jsonPath);
      } catch (IOException e) {
        throw new RuntimeException("Failed to load AprilTag layout: " + name, e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException("Failed to serialize AprilTag layout JSON for: " + name, e);
      }
    }
  }
}
