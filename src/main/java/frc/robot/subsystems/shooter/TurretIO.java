package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretInputs {}

  public default void setVolts(double volts) {}

  public default void setAngle(double angle) {}

  public default void updateInputs(TurretInputsAutoLogged inputs) {}
}
