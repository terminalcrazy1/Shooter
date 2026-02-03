package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.Constants.ControlSystemContext;
import frc.robot.subsystems.rollers.RollersConstants;
import java.util.Optional;

public final class IntakeConstants {

  public static final int ROLLER_CAN_ID = 14;
  public static final int PIVOT_CAN_ID = 17;
  public static final String CANBUS = "rio";

  public static final RollersConstants ROLLERS =
      new RollersConstants(16.0 / 24.0, false, 40, Units.inchesToMeters(1.0));

  public static final class Pivot {

    private static final ControlSystemConstants GAINS =
        new ControlSystemConstants(
            ControlSystemConstants.EMPTY_GAINS,
            new ControlSystemContext(
                0.2, // kS
                0.01, // kV
                1, // kA
                5.0, // kP
                0.0, // kD
                5 / 1.0, // gear ratio
                Optional.of(1.0),
                Optional.of(1.0)));

    public static ControlSystemContext getGains() {
      return GAINS.getConstants();
    }
  }

  private IntakeConstants() {}
}
