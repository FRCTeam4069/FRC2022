package frc.robot.components;

import frc.robot.Robot;
import static frc.robot.Constants.*;

/**
 * The Rear Intake component for FRC 2022
 */
public class RearIntake implements RobotComponent {

	private final Robot robot;

	/**
	 * @param robot Robot instance
	 */
	public RearIntake(Robot robot) {
		this.robot = robot;
	}

	// init function for the RearIntake
	@Override
	public RobotComponent init() {
		return this;
	}

	@Override
	public void loop() {}

	// Function to turn the component off
	@Override
	public void shutdown() {}
}
