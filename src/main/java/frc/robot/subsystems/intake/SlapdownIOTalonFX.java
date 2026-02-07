package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.pivot.PivotSpecifications;

public class SlapdownIOTalonFX extends PivotIOTalonFX {
  private final CANcoder cancoder;
  private final StatusSignal<Angle> absolutePositionSignal;

  private final double cancoderToMechanismRatio;

  @SuppressWarnings("removal")
  public SlapdownIOTalonFX(int canId, String canBus, PivotSpecifications constants) {
    super(canId, canBus, constants);

    cancoder = new CANcoder(IntakeConstants.Pivot.PIVOT_CANCODER_ID, canBus);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = IntakeConstants.Pivot.CANCODER_OFFSET_RAD / (2.0 * Math.PI);
    config.MagnetSensor.SensorDirection =
        IntakeConstants.Pivot.CANCODER_CLOCKWISE_POSITIVE
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> cancoder.getConfigurator().apply(config));

    absolutePositionSignal = cancoder.getAbsolutePosition();

    cancoderToMechanismRatio = IntakeConstants.Pivot.CANCODER_GEAR_RATIO;

    seedMotorFromCancoder();
  }

  private void seedMotorFromCancoder() {
    if (!BaseStatusSignal.refreshAll(absolutePositionSignal).isOK()) return;

    double cancoderRotations = absolutePositionSignal.getValue().in(Rotations);
    double mechanismRotations = cancoderRotations * cancoderToMechanismRatio;

    tryUntilOk(5, () -> motor.setPosition(mechanismRotations));
  }
}
