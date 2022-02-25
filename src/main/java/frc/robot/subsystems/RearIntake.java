package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Rear Intake Subsystem */
public class RearIntake implements RobotSubsystem {

	public static final int RI_NEO_DRIVE = 14;

	private CANSparkMax drive;

	@Override
	public void init() {
		drive = new CANSparkMax(RI_NEO_DRIVE, MotorType.kBrushless);
	}

	@Override
	public void loop() {
		
	}

	/**
     * Update desired articulation, driven percentage
     * @param drivenPercentage Between -1 and 1, percentage of driven power on intake/feed
     */
    public void update(double drivenPercentage) {
        drive.set(drivenPercentage);
    }

}
