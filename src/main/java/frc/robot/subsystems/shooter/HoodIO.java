package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodInputs {}

  public default void setVolts(double volts) {}

  public default void setAngle(double angle) {}

  public default void updateInputs(HoodInputsAutoLogged inputs) {}
}
