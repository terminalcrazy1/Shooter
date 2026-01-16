package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants.Flywheel;

public class FlywheelIOSim implements FlywheelIO {
  private double appliedVolts = 0.0;
  private double targetVelocityRadsPerSec = 0.0;
  private boolean usingVelocityTarget = false;

  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  public FlywheelIOSim() {
    DCMotor motor = DCMotor.getKrakenX60(1);
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motor, Flywheel.MOI, Flywheel.GEAR_RATIO), motor);
  }

  @Override
  public void setVolts(double volts) {
    this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    this.usingVelocityTarget = false;
  }

  @Override
  public void setVelocity(double velocityRadsPerSec) {
    this.targetVelocityRadsPerSec = velocityRadsPerSec;
    this.usingVelocityTarget = true;
  }

  @Override
  public void updateInputs(FlywheelIOInputsAutoLogged inputs) {
    if (this.usingVelocityTarget) {
      double pidOutput =
          this.pidController.calculate(
              motorSim.getAngularVelocityRadPerSec(), this.targetVelocityRadsPerSec);
      double feedforwardOutput =
          this.feedforwardController.calculate(this.targetVelocityRadsPerSec);

      this.setVolts(pidOutput + feedforwardOutput);
    }

    this.motorSim.setInputVoltage(appliedVolts);
    this.motorSim.update(0.020);

    inputs.appliedVoltage = this.appliedVolts;
    inputs.supplyCurrentAmps = this.motorSim.getCurrentDrawAmps();
    inputs.torqueCurrent = inputs.supplyCurrentAmps;
    inputs.velocityRadsPerSec = this.motorSim.getAngularVelocityRadPerSec();
    inputs.velocityMetersPerSec = inputs.velocityRadsPerSec * Flywheel.WHEEL_RADIUS_METERS;
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
