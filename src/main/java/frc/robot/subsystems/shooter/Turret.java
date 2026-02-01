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

  public Turret(PivotIO pivotIO) {
    super("Shooter/Turret", pivotIO);
  }

  public Command lockOntoTarget(Supplier<Angle> relativeAngleSupplier) {
    return this.runEnd(
        () -> {
          Angle relativeAngle = relativeAngleSupplier.get();

          // Calculate the nearest absolute angle

          this.io.setPosition(relativeAngle.in(Radians));
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
