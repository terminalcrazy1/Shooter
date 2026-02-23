package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TurretIO turretIO;
    private HoodIO hoodIO;

    public Shooter() {
        if (Constants.currentMode == Mode.SIM) {
            turretIO = new TurretIOSim();
            hoodIO = new HoodIOSim();
        } else if (Constants.currentMode == Mode.REAL){
            turretIO = new TurretIOTalonFX();
            hoodIO = new HoodIOTalonFX();
        }
    }

    public void setAngle(double turretAngle, double hoodAngle) {
        turretIO.setAngle(turretAngle);
        hoodIO.setAngle(hoodAngle);
    }
}
