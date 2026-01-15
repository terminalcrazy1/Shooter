package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim sim;
  private double appliedVoltage = 0;
  private double targetVelocityRadPerSec = 0;

  private SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.2, 0.01);
  PIDController feedbackController = new PIDController(5, 0, 0);

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
    // Velocity Control
    double ffVolts = feedforwardController.calculate(targetVelocityRadPerSec);
    double fbVolts =
        feedbackController.calculate(sim.getAngularVelocityRadPerSec(), targetVelocityRadPerSec);
    setVolts(ffVolts + fbVolts);
  }

  @Override
  public void setVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    targetVelocityRadPerSec = velocityRadPerSec;
  }

  @Override
  public void setControlConstants(double kS, double kV, double kP, double kD) {
    feedbackController.setPID(kP, 0, kD);
    feedforwardController = new SimpleMotorFeedforward(kS, kV);
  }
}
