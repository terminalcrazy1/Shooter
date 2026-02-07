package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Turret;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.print.attribute.standard.PrinterURI;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static enum SuperState {
    IDLE,
    READYING_SHOOTER,
    READY_TO_SHOOT,
    EXTENDING_CLIMBER,
    RETRACTING_CLIMBER,
    CLIMBER_DEPLOYED
  }

  public final StateMachine<SuperState> stateMachine = new StateMachine<>(SuperState.IDLE);

  private final Drive drive;
  private final Turret turret;
  private final Supplier<Pose2d> hubPoseSupplier;

  public Superstructure(Drive drive, Turret turret, Supplier<Pose2d> hubPoseSupplier) {
    this.drive = drive;
    this.turret = turret;
    this.hubPoseSupplier = hubPoseSupplier;

    configureStateRequirements();
    configureStateBehaviours();
  }

  /** Configure the requirements of each state */
  public void configureStateRequirements() {
    // Can always be set to idle
    stateMachine.stateRequirements.put(SuperState.IDLE, () -> true);
  }

  /** Configure what the behaviours of each state are */
  public void configureStateBehaviours() {
    // stateTriggers.get(SuperState.IDLE).onTrue(Command); Example Command
  }

  /** A command that requests a state for the state machine */
  public Command requestState(SuperState targetState) {
    return runOnce(() -> stateMachine.requestState(targetState));
  }

  /** A command that forces a state for the state machine */
  public Command forceState(SuperState targetState) {
    return runOnce(() -> stateMachine.forceState(targetState));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Superstructure/Components/TurretTarget",
        new Pose2d(
            drive
                .getPose()
                .getTranslation()
                .plus(
                    new Translation2d(
                        drive
                            .getPose()
                            .getTranslation()
                            .getDistance(hubPoseSupplier.get().getTranslation()),
                        drive.getRotation().plus(new Rotation2d(turret.getOrientation())))),
            Rotation2d.kZero));
  }
}
