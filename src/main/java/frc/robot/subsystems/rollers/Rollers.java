package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  public final String name;
  protected final RollersIO io;
  protected final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(String name, RollersIO io) {
    this.name = name;
    this.io = io;
  }

  public Command setVolts(double volts) {
    return this.runOnce(() -> io.setVolts(volts));
  }

  public Command setAngularVelocity(double velocityRadPerSec) {
    return this.runOnce(() -> io.setAngularVelocity(velocityRadPerSec));
  }

  public Command runVolts(double volts) {
    return this.runEnd(() -> io.setVolts(volts), () -> io.setVolts(0));
  }

  public Command runAngularVelocity(double velocityRadPerSec) {
    return this.runEnd(() -> io.setAngularVelocity(velocityRadPerSec), () -> io.stop());
  }

  public Command runLinearVelocity(double velocityMetersPerSec) {
    return this.runEnd(() -> io.setLinearVelocity(velocityMetersPerSec), () -> io.stop());
  }

  public Command runLinearVelocity(DoubleSupplier velocityMetersPerSecSupplier) {
    return this.runEnd(
        () -> io.setLinearVelocity(velocityMetersPerSecSupplier.getAsDouble()), () -> io.stop());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public RollersIO getIO() {
    return io;
  }

  public double getPositionRads() {
    return inputs.positionRads;
  }
}
