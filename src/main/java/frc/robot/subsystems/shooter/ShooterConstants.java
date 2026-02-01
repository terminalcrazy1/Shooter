package frc.robot.subsystems.shooter;

import frc.robot.Constants.ControlSystem;
import frc.robot.Constants.ControlSystemConstants;
import java.util.Optional;

public final class ShooterConstants {
  public static final class Flywheel {
    public static final int CAN_ID = 1;

    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kS = 0;

    public static final double kP = 0;
    public static final double kD = 0;

    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 1.0;

    public static final double MOI = 0.01;
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_RADIUS_METERS = 0.1;
  }

  public static final class TurretHeader {
    public static final int CAN_ID = 2;

    private static final ControlSystem GAINS =
        new ControlSystem(
            ControlSystem.EMPTY_GAINS,
            new ControlSystemConstants(
                4.44, 0.05, 0, 0, 40, 0, Optional.of(20.0), Optional.of(40.0)));

    public static ControlSystemConstants getGains() {
      return GAINS.getConstants();
    }

    public static final double MOI = 0.0648478785;
    public static final double GEAR_RATIO = 1 / 52.0;
    public static final double MIN_ANGLE_RADS = -200 * (Math.PI / 180);
    public static final double MAX_ANGLE_RADS = 200 * (Math.PI / 180);
  }

  public static final class TurretHood {
    public static final int CAN_ID = 3;

    public static final double kV = 0.1;
    public static final double kA = 0.1;
    public static final double kS = 0;

    public static final double kP = 0;
    public static final double kD = 0;

    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 1.0;
    public static final double MOI = 0.01;
    public static final double GEAR_RATIO = 1.0;
  }
}
