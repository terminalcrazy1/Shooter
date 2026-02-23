package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlywheelIOTalonFX implements FlywheelIO {
  private TalonFX forwardMotor = new TalonFX(19);
  private TalonFX reverseMotor = new TalonFX(20);

  @AutoLog
  class FlywheelInputs {}

  public void setVolts(double volts) {
    forwardMotor.setVoltage(volts);
    reverseMotor.setVoltage(-volts);
  }

  public double getVelocity(double velocity) {
    return forwardMotor.getVelocity().getValueAsDouble();
  }

  public void updateInputs(FlywheelInputsAutoLogged inputs) {}
}
