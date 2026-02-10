package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class BallTunneler extends Rollers {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("BallTunneler/kS", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("BallTunneler/kV", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("BallTunneler/kP", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("BallTunneler/kD", IndexerConstants.BallTunneler.SYSTEM_CONSTANTS.kD);

  public BallTunneler(RollersIO io) {
    super("BallTunneler", io);

    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  public Command runTunneler() {
    return runEnd(
        () ->
            io.setAngularVelocity(
                IndexerConstants.BallTunneler.TUNNELING_SPEED.in(RadiansPerSecond)),
        () -> io.stop());
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);

    super.periodic();
  }
}
