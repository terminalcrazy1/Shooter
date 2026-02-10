package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.function.Supplier;

public class ShootingUtil {
  public static Angle calculateHoodAngle(double distanceFromHub) {
    return Radians.of(
        MathUtil.clamp(
            distanceFromHub,
            ShooterConstants.Hood.MIN_ANGLE_RADS,
            ShooterConstants.Hood.MAX_ANGLE_RADS));
  }

  public static Angle calculateHoodAngle(
      Supplier<Pose2d> drivetrainPose, Supplier<Pose2d> targetPose) {
    return calculateHoodAngle(
        targetPose.get().getTranslation().getDistance(drivetrainPose.get().getTranslation()));
  }

  public static Angle calculateTurretRelativeAngle(
      Supplier<Pose2d> drivetrainPose, Supplier<Pose2d> targetPose) {
    Pose2d drivePose = drivetrainPose.get();
    Rotation2d driveHeading = drivePose.getRotation();
    Translation2d driveToHubVector =
        targetPose.get().getTranslation().minus(drivePose.getTranslation());
    Rotation2d pointToHubRotation =
        new Rotation2d(driveToHubVector.getX(), driveToHubVector.getY());

    return pointToHubRotation.minus(driveHeading).getMeasure();
  }
}
