package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersConstants;

public final class IntakeConstants {

  public static final int CAN_ID = 14;

  public static final RollersConstants ROLLERS =
      new RollersConstants(
          16.0 / 24.0, // gear ratio
          false, // inverted
          40, // current limit (amps)
          Units.inchesToMeters(1.0) // roller radius
          );

  private IntakeConstants() {}
}
