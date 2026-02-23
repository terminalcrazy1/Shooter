package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretIOTalonFX implements TurretIO {
  private TalonFX motor = new TalonFX(21);
  
  @AutoLog
  class HoodInputs {}

  public void setVolts(double volts) {
    motor.setVoltage(volts);
  }

  public void setAngle() {}

  public void updateInputs(HoodInputsAutoLogged inputs) {}
}
