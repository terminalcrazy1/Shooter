package frc.robot.subsystems.shooter;

import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.pivot.PivotSpecifications;
import java.util.Optional;

public final class ShooterConstants {
  public static final String CANBUS = "rio";

  public static final class Flywheel {
    public static final int MASTER_CAN_ID = 19;
    public static final int FOLLOWER_CAN_ID = 20;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.12, 0.05, 0.0, 0.0, 0.5, 0.0, Optional.empty(), Optional.empty()));

    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_RADIUS_METERS = 0.1;
  }

  public static final class Turret {
    public static final int CAN_ID = 21;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                1.0, 0.05, 0, 0, 20, 0.5, Optional.of(60.0), Optional.of(100.0)));

    public static final PivotSpecifications SPECS = new PivotSpecifications(52.0, false);

    public static final double MIN_ANGLE_RADS = -190 * (Math.PI / 180);
    public static final double MAX_ANGLE_RADS = 190 * (Math.PI / 180);
  }

  public static final class Hood {
    public static final int CAN_ID = 22;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                4.44, 0.1, 0.0, 0.0, 1.0, 0.0, Optional.of(1.0), Optional.of(1.0)));

    public static final PivotSpecifications SPECS = new PivotSpecifications(1.0, false);
  }
}
