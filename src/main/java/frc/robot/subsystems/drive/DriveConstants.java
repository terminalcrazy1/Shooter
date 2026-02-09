package frc.robot.subsystems.drive;

import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public final class DriveConstants {
  public static final LinearVelocity bumpModeMaxSpeed = TunerConstants.kSpeedAt12Volts.times(0.25);
  public static final LinearVelocity shootingModeMaxSpeed =
      TunerConstants.kSpeedAt12Volts.times(0.5);

  private DriveConstants() {}
}
