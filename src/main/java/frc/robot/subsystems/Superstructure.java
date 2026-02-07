package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.BallTunneler;
import frc.robot.subsystems.indexer.Serializer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  public static enum ShootingState {
    IDLE,
    READYING_SHOOTER,
    READY_TO_SHOOT,
    SHOOTING
  }

  public static enum IntakingState {
    STOWED,
    STOWING,
    DEPLOYING,
    INTAKE_READY,
    INTAKING
  }

  public final StateMachine<ShootingState> shootingStateMachine =
      new StateMachine<>(ShootingState.IDLE);
  public final StateMachine<IntakingState> intakingStateMachine =
      new StateMachine<>(IntakingState.STOWED);

  private final Drive drive;
  private final Intake intake;
  private final Serializer serializer;
  private final BallTunneler ballTunneler;
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Supplier<Pose2d> hubPoseSupplier;

  public Superstructure(
      Drive drive,
      Intake intake,
      Serializer serializer,
      BallTunneler ballTunneler,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Supplier<Pose2d> hubPoseSupplier) {
    this.drive = drive;
    this.intake = intake;
    this.serializer = serializer;
    this.ballTunneler = ballTunneler;
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
    this.hubPoseSupplier = hubPoseSupplier;

    configureStateRequirements();
    configureStateBehaviours();
  }

  /** Configure the requirements of each state */
  private void configureStateRequirements() {
    // Shooting
    shootingStateMachine.stateRequirements.put(
        ShootingState.IDLE, StateMachine.STATE_ALWAYS_AVAILABLE);
    shootingStateMachine.stateRequirements.put(
        ShootingState.READYING_SHOOTER, () -> shootingStateMachine.isInState(ShootingState.IDLE));
    shootingStateMachine.stateRequirements.put(
        ShootingState.READY_TO_SHOOT, () -> shootingStateMachine.isInState(ShootingState.SHOOTING));
    shootingStateMachine.stateRequirements.put(
        ShootingState.SHOOTING, () -> shootingStateMachine.isInState(ShootingState.READY_TO_SHOOT));

    // Intaking
    intakingStateMachine.stateRequirements.put(
        IntakingState.STOWING, StateMachine.STATE_ALWAYS_AVAILABLE);
    intakingStateMachine.stateRequirements.put(
        IntakingState.DEPLOYING, () -> !intakingStateMachine.isInState(IntakingState.INTAKING));
    intakingStateMachine.stateRequirements.put(
        IntakingState.INTAKE_READY, () -> intakingStateMachine.isInState(IntakingState.INTAKING));
    intakingStateMachine.stateRequirements.put(
        IntakingState.INTAKING, () -> intakingStateMachine.isInState(IntakingState.INTAKE_READY));
  }

  /** Configure what the behaviours of each state are */
  private void configureStateBehaviours() {
    shootingStateMachine.stateTriggers.get(ShootingState.IDLE).onTrue(flywheel.stop());

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.IDLE)
        .whileFalse(
            turret.lockOntoTarget(
                () -> {
                  Pose2d drivePose = this.drive.getPose();
                  Rotation2d driveHeading = drivePose.getRotation();
                  Translation2d driveToHubVector =
                      hubPoseSupplier.get().getTranslation().minus(drivePose.getTranslation());
                  Rotation2d pointToHubRotation =
                      new Rotation2d(driveToHubVector.getX(), driveToHubVector.getY());

                  return pointToHubRotation.minus(driveHeading).getMeasure();
                },
                () -> drive.getAngularVelocityRadPerSec()))
        .whileFalse(
            hood.trackTarget(
                () ->
                    Radians.of(
                        hubPoseSupplier
                            .get()
                            .getTranslation()
                            .getDistance(drive.getPose().getTranslation()))))
        .whileFalse(flywheel.runVelocityRadPerSec(5.0));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.READYING_SHOOTER)
        .and(flywheel.atTargetVelocity())
        .onTrue(forceState(ShootingState.READY_TO_SHOOT));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.SHOOTING)
        .whileTrue(ballTunneler.runTunneler());

    // Intaking state
    intakingStateMachine
        .stateTriggers
        .get(IntakingState.STOWING)
        .onTrue(intake.stow())
        .and(intake.pivotAtSetpoint())
        .onTrue(forceState(IntakingState.STOWED));

    intakingStateMachine.stateTriggers.get(IntakingState.DEPLOYING).onTrue(intake.deploy());

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.DEPLOYING)
        .and(intake.pivotAtSetpoint())
        .onTrue(forceState(IntakingState.INTAKE_READY));

    intakingStateMachine
        .stateTriggers
        .get(IntakingState.INTAKING)
        .whileTrue(
            intake.runLinearVelocity(() -> Math.max(drive.getLinearSpeedMetersPerSec() / 10, 1.0)));

    shootingStateMachine
        .stateTriggers
        .get(ShootingState.IDLE)
        .and(intakingStateMachine.stateTriggers.get(IntakingState.INTAKING).negate())
        .whileFalse(serializer.runSerializer());
  }

  /** A command that requests a state for the shooting state machine */
  public Command requestState(ShootingState targetState) {
    return runOnce(() -> shootingStateMachine.requestState(targetState));
  }

  /** A command that requests a state for the intaking state machine */
  public Command requestState(IntakingState targetState) {
    return runOnce(() -> intakingStateMachine.requestState(targetState));
  }

  /** Continuous requests a state until it is set or this command is interrupted */
  public Command continuouslyRequestState(ShootingState targetState) {
    return run(() -> shootingStateMachine.requestState(targetState))
        .until(() -> shootingStateMachine.isInState(targetState));
  }

  /** Continuous requests a state until it is set or this command is interrupted */
  public Command continuouslyRequestState(IntakingState targetState) {
    return run(() -> intakingStateMachine.requestState(targetState))
        .until(() -> intakingStateMachine.isInState(targetState));
  }

  /** A command that forces a state for the shooting state machine */
  public Command forceState(ShootingState targetState) {
    return runOnce(() -> shootingStateMachine.forceState(targetState));
  }

  /** A command that forces a state for the intaking state machine */
  public Command forceState(IntakingState targetState) {
    return runOnce(() -> intakingStateMachine.forceState(targetState));
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

    Logger.recordOutput("Superstructure/ShootingState", shootingStateMachine.getState().toString());
    Logger.recordOutput("Superstructure/IntakingState", intakingStateMachine.getState().toString());
  }
}
