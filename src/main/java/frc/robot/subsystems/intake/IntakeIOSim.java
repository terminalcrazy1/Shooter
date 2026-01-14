package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0;

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
}
