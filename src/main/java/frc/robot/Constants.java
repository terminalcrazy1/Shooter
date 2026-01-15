package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean disableHAL = currentMode != Mode.REAL;

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
}
