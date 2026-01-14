package frc.robot.subsystems.drive.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0;

  // Values based on a KrakenX60
  private final double Kt = 0.0166; // Nm/A
  private final double Kv = 55.65; // rads/s per volt
  private final double R = 12.0 / 85; // V/A

  public IntakeIOSim(DCMotor motor, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
  }

  @Override
  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setTorqueNm(double torqueNm) {
    double omega = sim.getAngularVelocityRadPerSec();

    double targetCurrent = torqueNm / Kt;
    double volts = targetCurrent * R + omega / Kv;
    setVolts(MathUtil.clamp(volts, -12, 12));
  }
}
