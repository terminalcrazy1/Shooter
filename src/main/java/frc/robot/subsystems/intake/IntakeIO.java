package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  // velocity and position are measured in mechanism units not motor units
  public static class IntakeIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrent = 0.0;
    public boolean connected = false;
  }

  default void setVolts(double volts) {}

  default void setVelocity(double velocityRadsPerSec) {}

  default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  default void setControlConstants(double kS, double kV, double kP, double kD) {}
}
