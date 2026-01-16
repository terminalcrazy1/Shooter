package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  // velocity and position are measured in mechanism units not motor units
  public static class FlywheelIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrent = 0.0;
    public boolean connected = false;
  }

  public default void setVolts(double volts) {}

  public default void setVelocity(double velocityRadsPerSec) {}

  public default void updateInputs(FlywheelIOInputsAutoLogged inputs) {}

  public default void setControlConstants(double kS, double kV, double kA, double kP, double kD) {}

  public default void setMotionProfile(double maxVelocity, double maxAcceleration) {}
}
