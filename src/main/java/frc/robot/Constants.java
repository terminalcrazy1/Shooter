package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Allow tuning in SIM or REAl, disallow in a competition match or REPLAY
  public static boolean tuningMode() {
    switch (currentMode) {
      case SIM:
        return true;

      case REAL:
        return !DriverStation.isFMSAttached();

      case REPLAY:
      default:
        return false;
    }
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public record ControlSystemContext(
      double kV,
      double kA,
      double kS,
      double kG,
      double kP,
      double kD,
      Optional<Double> maxVelocity,
      Optional<Double> maxAcceleration) {}

  public static class ControlSystemConstants {
    public static final ControlSystemContext EMPTY_GAINS =
        new ControlSystemContext(0, 0, 0, 0, 0, 0, Optional.empty(), Optional.empty());
    public final ControlSystemContext REAL_GAINS;
    public final ControlSystemContext SIM_GAINS;

    public ControlSystemConstants(ControlSystemContext realGains, ControlSystemContext simGains) {
      REAL_GAINS = realGains;
      SIM_GAINS = simGains;
    }

    public ControlSystemContext getConstants() {
      switch (currentMode) {
        case REAL:
          return REAL_GAINS;
        case SIM:
          return SIM_GAINS;
        default:
          return EMPTY_GAINS;
      }
    }
  }
}
