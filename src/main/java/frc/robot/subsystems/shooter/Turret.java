package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlSystemConstants;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.shooter.ShooterConstants.TurretHeader;
import frc.robot.subsystems.shooter.ShooterConstants.TurretHood;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final PivotIO turretHeaderIO;
  private final PivotIO turretHoodIO;

  private final PivotIOInputsAutoLogged turretHeaderIOInputs = new PivotIOInputsAutoLogged();
  private final PivotIOInputsAutoLogged turretHoodIOInputs = new PivotIOInputsAutoLogged();

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

  private final LoggedTunableNumber turretHoodKs =
      new LoggedTunableNumber("Shooter/TurretHood/kS", TurretHood.kS);
  private final LoggedTunableNumber turretHoodKv =
      new LoggedTunableNumber("Shooter/TurretHood/kV", TurretHood.kV);
  private final LoggedTunableNumber turretHoodKa =
      new LoggedTunableNumber("Shooter/TurretHood/kA", TurretHood.kA);
  private final LoggedTunableNumber turretHoodKp =
      new LoggedTunableNumber("Shooter/TurretHood/kP", TurretHood.kP);
  private final LoggedTunableNumber turretHoodKd =
      new LoggedTunableNumber("Shooter/TurretHood/kD", TurretHood.kD);

  private final LoggedTunableNumber turretHoodMaxVelocity =
      new LoggedTunableNumber("Shooter/TurretHood/MaxVelocity", TurretHood.MAX_VELOCITY);
  private final LoggedTunableNumber turretHoodMaxAcceleration =
      new LoggedTunableNumber("Shooter/TurretHood/MaxAcceleration", TurretHood.MAX_ACCELERATION);

  public Turret(PivotIO turretHeaderIO, PivotIO turretHoodIO) {
    this.turretHeaderIO = turretHeaderIO;
    this.turretHoodIO = turretHoodIO;

    this.turretHeaderIO.setControlConstants(
        turretHeaderKs.get(),
        turretHeaderKv.get(),
        turretHeaderKa.get(),
        turretHeaderKp.get(),
        turretHeaderKd.get());
    this.turretHeaderIO.setMotionProfile(
        turretHeaderMaxVelocity.get(), turretHeaderMaxAcceleration.get());

    this.turretHoodIO.setControlConstants(
        turretHoodKs.get(),
        turretHoodKv.get(),
        turretHoodKa.get(),
        turretHoodKp.get(),
        turretHoodKd.get());
    this.turretHoodIO.setMotionProfile(
        turretHoodMaxVelocity.get(), turretHoodMaxAcceleration.get());
  }

  public Command lockOntoTarget(
      Supplier<Angle> headingSupplier, Supplier<Angle> hoodAngleSupplier) {
    return this.runEnd(
        () -> {
          Angle heading = headingSupplier.get();
          Angle hoodAngle = hoodAngleSupplier.get();

          Logger.recordOutput("Shooter/Target/Heading", heading);
          Logger.recordOutput("Shooter/Target/HoodAngle", hoodAngle);

          this.turretHeaderIO.setPosition(heading.in(Radians));
          //   this.turretHoodIO.setPosition(hoodAngle.in(Radians));
        },
        () -> {
          this.turretHeaderIO.stop();
        });
  }

  public Rotation2d getPivotRotation() {
    return Rotation2d.fromRadians(this.turretHeaderIOInputs.positionRads);
  }

  @Override
  public void periodic() {
    this.turretHeaderIO.updateInputs(turretHeaderIOInputs);
    this.turretHoodIO.updateInputs(turretHoodIOInputs);

    Logger.processInputs("Shooter/TurretHeader", turretHeaderIOInputs);
    Logger.processInputs("Shooter/TurretHood", turretHoodIOInputs);

    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id,
        (constants) ->
            this.turretHeaderIO.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        turretHeaderKs,
        turretHeaderKv,
        turretHeaderKa,
        turretHeaderKp,
        turretHeaderKd);
    LoggedTunableNumber.ifChanged(
        id,
        (constants) -> this.turretHeaderIO.setMotionProfile(constants[0], constants[1]),
        turretHeaderMaxVelocity,
        turretHeaderMaxAcceleration);

    LoggedTunableNumber.ifChanged(
        id,
        (constants) ->
            this.turretHoodIO.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        turretHoodKs,
        turretHoodKv,
        turretHoodKa,
        turretHoodKp,
        turretHoodKd);
    LoggedTunableNumber.ifChanged(
        id,
        (constants) -> this.turretHoodIO.setMotionProfile(constants[0], constants[1]),
        turretHoodMaxVelocity,
        turretHoodMaxAcceleration);
  }
}
