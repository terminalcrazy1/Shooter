package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.shooter.ShooterConstants.TurretHeader;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends Pivot {
  private final ControlSystemConstants turretHeaderGains = TurretHeader.getGains();
  private final LoggedTunableNumber turretHeaderKs =
      new LoggedTunableNumber("Shooter/TurretHeader/kS", turretHeaderGains.kS());
  private final LoggedTunableNumber turretHeaderKv =
      new LoggedTunableNumber("Shooter/TurretHeader/kV", turretHeaderGains.kV());
  private final LoggedTunableNumber turretHeaderKa =
      new LoggedTunableNumber("Shooter/TurretHeader/kA", turretHeaderGains.kA());
  private final LoggedTunableNumber turretHeaderKp =
      new LoggedTunableNumber("Shooter/TurretHeader/kP", turretHeaderGains.kP());
  private final LoggedTunableNumber turretHeaderKd =
      new LoggedTunableNumber("Shooter/TurretHeader/kD", turretHeaderGains.kD());

  private final LoggedTunableNumber turretHeaderMaxVelocity =
      new LoggedTunableNumber(
          "Shooter/TurretHeader/MaxVelocity", turretHeaderGains.maxVelocity().get());
  private final LoggedTunableNumber turretHeaderMaxAcceleration =
      new LoggedTunableNumber(
          "Shooter/TurretHeader/MaxAcceleration", turretHeaderGains.maxAcceleration().get());

  private final double FULL_CIRCLE_RADS = Math.PI * 2;

  public Turret(PivotIO pivotIO) {
    super("Shooter/Turret", pivotIO);
  }

  public Command lockOntoTarget(
      Supplier<Angle> relativeAngleSupplier, Supplier<Double> driveOmegaRadPerSecSupplier) {
    return this.runEnd(
        () -> {
          double relativeAngleRads = relativeAngleSupplier.get().in(Radians);
          Logger.recordOutput("Turret/RelativeAngleOffset", relativeAngleRads);

          // Calculate the nearest plausible absolute angle
          double positiveTargetAngleRads =
              (relativeAngleRads >= 0) ? relativeAngleRads : FULL_CIRCLE_RADS + relativeAngleRads;
          double negativeTargetAngleRads =
              (relativeAngleRads < 0) ? relativeAngleRads : -FULL_CIRCLE_RADS + relativeAngleRads;

          double absoluteAngleRads;

          if (positiveTargetAngleRads > TurretHeader.MAX_ANGLE_RADS) {
            absoluteAngleRads = negativeTargetAngleRads;
          } else if (negativeTargetAngleRads < TurretHeader.MIN_ANGLE_RADS) {
            absoluteAngleRads = positiveTargetAngleRads;
          } else {
            double errorFromPositiveAngle =
                Math.abs(positiveTargetAngleRads - inputsAutoLogged.positionRads);
            double errorFromNegativeAngle =
                Math.abs(negativeTargetAngleRads - inputsAutoLogged.positionRads);
            absoluteAngleRads =
                (errorFromPositiveAngle < errorFromNegativeAngle)
                    ? positiveTargetAngleRads
                    : negativeTargetAngleRads;
          }

          Logger.recordOutput("Turret/PositiveTarget", positiveTargetAngleRads);
          Logger.recordOutput("Turret/NegativeTarget", negativeTargetAngleRads);
          Logger.recordOutput("Turret/AbsoluteTargetAngle", absoluteAngleRads);

          this.io.setPositionWithExtraOmega(absoluteAngleRads, -driveOmegaRadPerSecSupplier.get());
        },
        () -> {
          this.io.stop();
        });
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id,
        (constants) ->
            this.io.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        turretHeaderKs,
        turretHeaderKv,
        turretHeaderKa,
        turretHeaderKp,
        turretHeaderKd);
    LoggedTunableNumber.ifChanged(
        id,
        (constants) -> this.io.setMotionProfile(constants[0], constants[1]),
        turretHeaderMaxVelocity,
        turretHeaderMaxAcceleration);

    super.periodic();
  }
}
