package frc.robot.subsystems.intake;

import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends Rollers {

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.2);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.01);
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 5.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.0);

  public Intake(RollersIO io) {
    super("Intake", io);
  }

  @Override
  public void periodic() {
    int id = hashCode();

    LoggedTunableNumber.ifChanged(
        id, c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]), kS, kV, kP, kD);
    super.periodic();
  }
}
