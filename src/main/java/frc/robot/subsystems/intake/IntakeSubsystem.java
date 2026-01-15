package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.5);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.5);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 1);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0);

  public IntakeSubsystem(IntakeIO io) {

    this.io = io;
    io.setControlConstants(kS.get(), kV.get(), kP.get(), kD.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
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
  public Command runVoltsIntake(double inputVolts) {
    return startEnd(() -> io.setVolts(inputVolts), () -> io.setVolts(0));
  }

  public void setIntakeVelocity(double velocityRadsPerSec) {
    io.setVelocity(velocityRadsPerSec);
  }
}
