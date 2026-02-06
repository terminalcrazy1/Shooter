package frc.robot.subsystems.indexer;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Serializer extends Rollers {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber(
          "Serializer/kS", IndexerConstants.Serializer.SYSTEM_CONSTANTS.getConstants().kS());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber(
          "Serializer/kV", IndexerConstants.Serializer.SYSTEM_CONSTANTS.getConstants().kV());
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber(
          "Serializer/kP", IndexerConstants.Serializer.SYSTEM_CONSTANTS.getConstants().kP());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber(
          "Serializer/kD", IndexerConstants.Serializer.SYSTEM_CONSTANTS.getConstants().kD());

  public Serializer(RollersIO io) {
    super("Serializer", io);
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);

    super.periodic();
  }
}
