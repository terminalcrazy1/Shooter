package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  // Existing mode fields
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Add a derived constant for disabling HAL
  public static final boolean disableHAL = currentMode != Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
