package frc.robot.subsystems.balltunneler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class BallTunneler extends SubsystemBase {
  private final BallTunnelerIO io;
  private final BallTunnelerIOInputsAutoLogged inputs = new BallTunnelerIOInputsAutoLogged();

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("BallTunnelerConstants/kS", BallTunnelerConstants.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("BallTunnelerConstants/kV", BallTunnelerConstants.kV);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("BallTunnelerConstants/kP", BallTunnelerConstants.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("BallTunnelerConstants/kD", BallTunnelerConstants.kD);

  public BallTunneler(BallTunnelerIO io) {

    this.io = io;
    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("BallTunneler", inputs);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        (constants) ->
            io.setControlConstants(constants[0], constants[1], constants[2], constants[3]),
        kS,
        kV,
        kP,
        kD);
  }

  @AutoLogOutput
  public Command runVoltsSerializer(double inputVolts) {
    return startEnd(() -> io.setVolts(inputVolts), () -> io.setVolts(0));
  }

  public Command runSerializeVelocityRadsPerSec(double velocityRadsPerSec) {

    return startEnd(() -> io.setVelocity(velocityRadsPerSec), () -> io.setVelocity(0));
  }
}
