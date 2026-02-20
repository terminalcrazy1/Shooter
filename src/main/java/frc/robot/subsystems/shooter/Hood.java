package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface Hood {
  @AutoLog
  class HoodInputs {}

  public default void setVolts() {}

  public default void setAngle() {}

  public default void updateInputs(HoodInputsAutoLogged inputs) {}
}
