package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOTalonFX implements HoodIO {
  private TalonFX motor = new TalonFX(23);
  
  @AutoLog
  class HoodInputs {}

  public void setVolts(double volts) {
    motor.setVoltage(volts);
  }

  public void setAngle(double angle) {
    double motorAngle = motor.getPosition().getValueAsDouble();
    double hoodAngle = (motorAngle * 360) / 96;
    if (angle != hoodAngle) {
      // do pid here
    }
  }

  public void updateInputs(HoodInputsAutoLogged inputs) {}
}
