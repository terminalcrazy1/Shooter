package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface Turret {
  @AutoLog
  class TurretInputs {}

  public default void setVolts(double volts) {}

  public default void setAngle() {}

  public default void updateInputs(TurretInputsAutoLogged inputs) {}
}
