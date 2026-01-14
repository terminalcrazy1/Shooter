package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.intake.IntakeIOInputsAutoLogged;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public boolean connected = false;
  }

  default void setVolts(double volts) {}

  default void setTorqueNm(double torqueNm) {}

  default void updateInputs(IntakeIOInputsAutoLogged inputs) {}
}
