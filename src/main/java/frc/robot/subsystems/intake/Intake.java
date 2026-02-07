package frc.robot.subsystems.intake;

import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends Rollers {
    
  private final LoggedTunableNumber rollers_kS = 
  new LoggedTunableNumber("Intake/kS",  IntakeConstants.Rollers.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber rollers_kV = 
  new LoggedTunableNumber("Intake/kV", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber rollers_kP = 
  new LoggedTunableNumber("Intake/kP", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber rollers_kD = 
  new LoggedTunableNumber("Intake/kD", IntakeConstants.Rollers.SYSTEM_CONSTANTS.kD);

  public final Pivot pivot;
  private final LoggedTunableNumber pivot_kS =
      new LoggedTunableNumber("IntakePivot/kS", IntakeConstants.Pivot.SYSTEM_CONSTANTS.kS);
  private final LoggedTunableNumber pivot_kV =
      new LoggedTunableNumber("IntakePivot/kV", IntakeConstants.Pivot.SYSTEM_CONSTANTS.kV);
  private final LoggedTunableNumber pivot_kA =
      new LoggedTunableNumber("IntakePivot/kA", IntakeConstants.Pivot.SYSTEM_CONSTANTS.kA);
  private final LoggedTunableNumber pivot_kP =
      new LoggedTunableNumber("IntakePivot/kP", IntakeConstants.Pivot.SYSTEM_CONSTANTS.kP);
  private final LoggedTunableNumber pivot_kD =
      new LoggedTunableNumber("IntakePivot/kD", IntakeConstants.Pivot.SYSTEM_CONSTANTS.kD);

  public Intake(RollersIO rollersIO, PivotIO pivotIO) {
    super("Intake", rollersIO);
    this.pivot = new Pivot("IntakePivot", pivotIO);

    rollersIO.setControlConstants(
        rollers_kS.get(), rollers_kV.get(), rollers_kP.get(), rollers_kD.get());
    pivotIO.setControlConstants(
        pivot_kS.get(), pivot_kV.get(), pivot_kA.get(), pivot_kP.get(), pivot_kD.get());
  }

  @Override
  public void periodic() {
    int id = hashCode();

    // Update roller control constants if changed
    LoggedTunableNumber.ifChanged(
        id,
        c -> getIO().setControlConstants(c[0], c[1], c[2], c[3]),
        rollers_kS,
        rollers_kV,
        rollers_kP,
        rollers_kD);

    // Update pivot control constants if changed
    LoggedTunableNumber.ifChanged(
        id,
        c -> pivot.getIO().setControlConstants(c[0], c[1], c[2], c[3], c[4]),
        pivot_kS,
        pivot_kV,
        pivot_kA,
        pivot_kP,
        pivot_kD);

    super.periodic();
  }
}
