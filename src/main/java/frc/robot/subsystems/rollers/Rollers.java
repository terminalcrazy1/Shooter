package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  public final String name;
  protected final RollersIO io;
  protected final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(String name, RollersIO io) {
    this.name = name;
    this.io = io;
  }

  public Command runVolts(double volts) {
    return this.runOnce(() -> io.setVolts(volts));
  }

  public Command runVelocity(double velocityRadPerSec) {
    return this.runOnce(() -> io.setVelocity(velocityRadPerSec));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public RollersIO getIO() {
    return io;
  }
}
