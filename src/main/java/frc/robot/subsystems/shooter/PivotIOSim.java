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
  private boolean stopped = false;

  private final DCMotorSim motorSim;

  private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(0.0, 0.0);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  // private static final LoggedTunableNumber testKv = new LoggedTunableNumber("Test/kV",
  // 0.000365274);
  // private static final LoggedTunableNumber testKa = new LoggedTunableNumber("Test/kA", 5.707);

  public PivotIOSim(DCMotor motor, double moi, double gearRatio) {
    this.motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, gearRatio), motor);
  }

  @Override
  public void setVolts(double volts) {
    this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    this.usingTarget = false;
    this.stopped = false;
  }

  public void setVolts(double volts, boolean useTargetRotation) {
    this.appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    this.usingTarget = useTargetRotation;
    this.stopped = false;
  }

  @Override
  public void setPosition(double angleRads) {
    this.targetAngleRads = angleRads;
    usingTarget = true;
    this.stopped = false;
  }

  @Override
  public void stop() {
    this.appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     (constants) -> {
    //       DCMotorSim testSim =
    //           new DCMotorSim(
    //               LinearSystemId.createDCMotorSystem(constants[0], constants[1]),
    //               DCMotor.getKrakenX60(1));
    //       System.out.printf(
    //           "%s | %s\n",
    //           testSim.getJKgMetersSquared() - TurretHeader.MOI,
    //           testSim.getGearing() - TurretHeader.GEAR_RATIO);
    //     },
    //     testKv,
    //     testKa);

    if (!this.stopped) {
      if (this.usingTarget) {
        double angRad = motorSim.getAngularPositionRad();

        double pidOutput = this.pidController.calculate(angRad, this.targetAngleRads);
        double feedforwardOutput = this.feedforwardController.calculate(pidOutput);

        this.setVolts(pidOutput + feedforwardOutput, false);
      }

      this.motorSim.setInputVoltage(this.appliedVolts);
      this.motorSim.update(0.020);
    }

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
