package frc.robot.subsystems.serializer;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.rollers.RollersSpecifications;

public final class SerializerConstants {

  public static final int CAN_ID = 15;

  public static final RollersSpecifications ROLLERS =
      new RollersSpecifications(5, true, 40, Units.inchesToMeters(8));

  private SerializerConstants() {}
}
