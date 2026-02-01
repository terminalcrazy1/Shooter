package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.PivotIOInputsAutoLogged;

public class Pivot extends SubsystemBase {
	public final String name;

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputsAutoLogged = new PivotIOInputsAutoLogged();

	public Pivot(String name, PivotIO io) {
		this.name = name;
		this.io = io;
	}

	public Command setVoltage(double voltage) {
		return this.runOnce(() -> io.setVolts(voltage));
	}

	public Command setTargetAngle(Angle target) {
		return this.runOnce(() -> io.setPosition(target.in(Radians)));
	}

	public double getOrientationRads() {
		return inputsAutoLogged.positionRads;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputsAutoLogged);
		Logger.processInputs(name, inputsAutoLogged);
	}
}
