package frc.robot.components;

import frc.robot.Robot;
import static frc.robot.Constants.*;

/**
 * The Rear Intake component for FRC 2022
 */
public class RearIntake implements RobotComponent {
	private Robot robot;

	// init function for the RearIntake
	@Override
	public RobotComponent init(Robot robot) {
		this.robot = robot;

		return this;
	}

	// Function to turn the component off
	@Override
	public void shutdown() {}
}
