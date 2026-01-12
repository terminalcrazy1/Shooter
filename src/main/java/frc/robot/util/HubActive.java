package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.Random;

public class HubActive {
  private static Random rand = new Random();
  private static Boolean HubState = rand.nextBoolean();

  public static Boolean isCurAllianceActiveFirst() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
      return null;
    }

    char firstChar = gameData.charAt(0);
    Optional<DriverStation.Alliance> optionalAlliance = DriverStation.getAlliance();

    if (optionalAlliance.isEmpty()) {
      return null;
    }

    DriverStation.Alliance myAlliance = optionalAlliance.get();

    return (myAlliance == DriverStation.Alliance.Red && firstChar == 'R')
        || (myAlliance == DriverStation.Alliance.Blue && firstChar == 'B');
  }

  public static Boolean randomCurrentAllianceActiveFirst() {
    return HubState;
  }
}
