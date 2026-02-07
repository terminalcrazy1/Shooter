package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.rollers.RollersSpecifications;
import java.util.Optional;

public final class IntakeConstants {

  public static final String CANBUS = "rio";

  public static final class Rollers {
    public static final int CAN_ID = 14;

    public static final RollersSpecifications SPECS =
        new RollersSpecifications(
            16.0 / 24.0,
            false,
            40,
            Units.inchesToMeters(1.0));
  }

  public static final class Pivot {
    public static final int CAN_ID = 17;
    public static final int PIVOT_CANCODER_ID = 18;

    public static final boolean MOTOR_CLOCKWISE_POSITIVE = true;
    public static final boolean CANCODER_CLOCKWISE_POSITIVE = true;

    public static final double CANCODER_GEAR_RATIO = 1.0;
    public static final double CANCODER_OFFSET_RAD = 0.0;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.12,  // kS
                0.05,  // kV
                0.0,   // kA
                0.0,   // kP
                1.0,   // kD
                0.4,
                Optional.of(5.0),
                Optional.of(20.0)));
  }

  private IntakeConstants() {}
}