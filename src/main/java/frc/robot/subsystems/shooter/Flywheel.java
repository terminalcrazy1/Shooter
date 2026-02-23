package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface Flywheel {
  @AutoLog
  class FlywheelInputs {}

  public default void setVolts(double volts) {}

  public default void setVelocity() {}

  public default void updateInputs(FlywheelInputsAutoLogged inputs) {}
}
