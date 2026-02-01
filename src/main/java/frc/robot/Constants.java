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

  public record ControlSystemConstants(
      double kV,
      double kA,
      double kS,
      double kG,
      double kP,
      double kD,
      Optional<Double> maxVelocity,
      Optional<Double> maxAcceleration) {}

  public static class ControlSystem {
    public static final ControlSystemConstants EMPTY_GAINS =
        new ControlSystemConstants(0, 0, 0, 0, 0, 0, Optional.empty(), Optional.empty());
    private final ControlSystemConstants rModeGains;
    private final ControlSystemConstants sModeGains;

    public ControlSystem(ControlSystemConstants realGains, ControlSystemConstants simGains) {
      rModeGains = realGains;
      sModeGains = simGains;
    }

    public ControlSystemConstants getConstants() {
      switch (currentMode) {
        case REAL:
          return rModeGains;
        case SIM:
          return sModeGains;
        default:
          return EMPTY_GAINS;
      }
    }
  }
}
