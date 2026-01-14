package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @AutoLogOutput
  public Command runVoltsIntake(double inputVolts) {
    return startEnd(() -> io.setVolts(inputVolts), () -> io.setVolts(0));
  }

  public Command runVelocityIntake(double inputVelocity) {
    return startEnd(() -> io.setVelocity(inputVelocity), () -> io.setVelocity(0));
  }
}
