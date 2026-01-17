package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

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

  public record ModeGains(double kV, double kA, double kS, double kG, double kP, double kD) {}

  public static class ControlSystemGains {
    public static final ModeGains EMPTY_GAINS = new ModeGains(0, 0, 0, 0, 0, 0);
    ModeGains rModeGains;
    ModeGains sModeGains;

    public ControlSystemGains(ModeGains realGains, ModeGains simGains) {
      rModeGains = realGains;
      sModeGains = simGains;
    }

    public ModeGains getGains() {
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
