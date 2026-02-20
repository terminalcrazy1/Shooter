package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface Flywheel {
  @AutoLog
  class FlywheelInputs {}

  public default void setVolts() {}

  public default void setVelocity() {}

  public default void updateInputs(FlywheelInputsAutoLogged inputs) {}
}
