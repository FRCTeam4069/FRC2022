package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Rear Intake Subsystem */
public class RearIntake {

	private static final int DRIVE = 14;
    private static final double DRIVE_MAGNITUDE = 1;

	private CANSparkMax drive;

	public RearIntake() {
		drive = new CANSparkMax(DRIVE, MotorType.kBrushless);
	}

    /**
     * Update the drive state of the rear intake
     * 
     * @param enabled Rear intake on, off
     * @param dir Reverse direction
     */
    public void drive(boolean enabled, boolean reverse) {
        if (!enabled) drive.set(0);
        else if (!reverse) drive.set(DRIVE_MAGNITUDE);
        else drive.set(-DRIVE_MAGNITUDE);
    }

}
