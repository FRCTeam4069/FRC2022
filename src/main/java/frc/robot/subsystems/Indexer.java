package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Robot Indexer Subsystem */
public class Indexer {

    private static final int DRIVE_CAN = 21;
    private static final double DRIVE_MAGNITUDE = 1;

    private final CANSparkMax drive;

    public Indexer() {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
    }

    /**
     * Update the drive state of the indexer
     * 
     * @param enabled Indexer on, off
     * @param dir Reverse direction
     */
    public void drive(boolean enabled, boolean reverse) {
        if (!enabled) drive.set(0);
        else if (!reverse) drive.set(DRIVE_MAGNITUDE);
        else drive.set(-DRIVE_MAGNITUDE);
    }

}
