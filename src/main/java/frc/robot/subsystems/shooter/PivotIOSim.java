package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotIOSim implements PivotIO {
  private double appliedVolts = 0.0;
  private double targetAngleRads = 0.0;
  private boolean usingTarget = false;

  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  public PivotIOSim(double kV, double kA, DCMotor motor) {
    this.motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), motor);
  }

  @Override
  public void setVolts(double volts) {
    this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    this.usingTarget = false;
  }

  @Override
  public void setPosition(double angleRads) {
    this.targetAngleRads = angleRads;
    usingTarget = true;
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    if (this.usingTarget) {
      double pidOutput =
          this.pidController.calculate(motorSim.getAngularPositionRad(), this.targetAngleRads);
      double feedforwardOutput = this.feedforwardController.calculate(pidOutput);

      this.setVolts(pidOutput + feedforwardOutput);
    }

    this.motorSim.setInputVoltage(this.appliedVolts);
    this.motorSim.update(0.020);

    inputs.appliedVoltage = this.appliedVolts;
    inputs.supplyCurrentAmps = this.motorSim.getCurrentDrawAmps();
    inputs.torqueCurrent = inputs.supplyCurrentAmps;
    inputs.velocityRadsPerSec = this.motorSim.getAngularVelocityRadPerSec();
    inputs.positionRads = this.motorSim.getAngularPositionRad();
  }

  @Override
  public void setControlConstants(double kS, double kV, double kA, double kP, double kD) {
    this.feedforwardController.setKa(kA);
    this.feedforwardController.setKs(kS);
    this.feedforwardController.setKv(kV);

    this.pidController.setPID(kP, 0.0, kD);
  }

  @Override
  public void setMotionProfile(double maxVelocity, double maxAcceleration) {
    this.pidController.setConstraints(new Constraints(maxVelocity, maxAcceleration));
  }
}
