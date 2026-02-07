package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.pivot.PivotSpecifications;
import frc.robot.subsystems.rollers.RollersSpecifications;
import java.util.Optional;

public final class IntakeConstants {

  public static final String CANBUS = "rio";

  public static final class Rollers {
    public static final int CAN_ID = 14;

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.01, 0.001, 0.2, 0.0, 5.0, 0.0, Optional.empty(), Optional.empty()));

    public static final RollersSpecifications SPECS =
        new RollersSpecifications(16.0 / 24.0, false, 40, Units.inchesToMeters(1.0));
  }

  public static final class Pivot {
    public static final int MOTOR_CAN_ID = 15;
    public static final int CANCODER_ID = 16;

    public static final boolean CANCODER_CLOCKWISE_POSITIVE = true;

    public static final double CANCODER_GEAR_RATIO = 1.0;
    public static final double CANCODER_OFFSET_RAD = 0.0;

    public static final Angle STOWED_ANGLE = Degrees.zero();
    public static final Angle DEPLOYED_ANGLE = Degrees.of(90);

    public static final ControlSystemConstants SYSTEM_CONSTANTS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_CONTEXT,
            new ControlSystemContext(
                0.12, // kS
                0.05, // kV
                0.0, // kA
                0.0, // kP
                1.0, // kD
                0.4,
                Optional.of(5.0),
                Optional.of(20.0)));

    public static final PivotSpecifications SPECS = new PivotSpecifications(1.0, true);
  }

  private IntakeConstants() {}
}
