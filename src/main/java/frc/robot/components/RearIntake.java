package frc.robot.components;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Rear intake component
 */
public class RearIntake implements RobotComponent {

	/*
	Front and rear intake are functionally similar.

	Code 'borrowed' from the FrontIntake class.
	(Sorry to those assigned components, we needed this to prototype.)
	 */

	private CANSparkMax drive;

	// init function for the RearIntake
	@Override
	public RobotComponent init() {
		drive = new CANSparkMax(RI_NEO_DRIVE, MotorType.kBrushless);
		return this;
	}

	/**
     * Update desired articulation, driven percentage
     * @param drivenPercentage Between -1 and 1, percentage of driven power on intake/feed
     */
    public void update(double drivenPercentage) {
        drive.set(drivenPercentage);
    }

	@Override
	public void loop() {
		
	}
}
