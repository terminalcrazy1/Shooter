package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class Hood extends Pivot {
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Shooter/Hood/kS", ShooterConstants.Hood.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Shooter/Hood/kV", ShooterConstants.Hood.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Shooter/Hood/kA", ShooterConstants.Hood.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/Hood/kP", ShooterConstants.Hood.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/Hood/kD", ShooterConstants.Hood.SYSTEM_CONSTANTS.kD);

  private final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber(
          "Shooter/Hood/MaxVelocity", ShooterConstants.Hood.SYSTEM_CONSTANTS.maxVelocity.get());
  private final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Shooter/Hood/MaxAcceleration",
          ShooterConstants.Hood.SYSTEM_CONSTANTS.maxAcceleration.get());

  public Hood(PivotIO io) {
    super("Shooter/Hood", io);

    io.setControlConstants(kS.get(), kV.get(), kA.get(), kP.get(), kD.get());
    io.setMotionProfile(maxVelocity.get(), maxAcceleration.get());
  }

  public Command trackTarget(Supplier<Angle> angleSupplier) {
    return runEnd(
        () -> {
          io.setPosition(angleSupplier.get().in(Radians));
        },
        () -> io.stop());
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id,
        (constants) ->
            this.io.setControlConstants(
                constants[0], constants[1], constants[2], constants[3], constants[4]),
        kS,
        kV,
        kA,
        kP,
        kD);
    LoggedTunableNumber.ifChanged(
        id,
        (constants) -> this.io.setMotionProfile(constants[0], constants[1]),
        maxVelocity,
        maxAcceleration);

    super.periodic();
  }
}
