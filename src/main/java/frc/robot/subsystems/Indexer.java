package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Robot Indexer Subsystem */
public class Indexer {

    private static final int DRIVE_CAN = 21;

    private final CANSparkMax drive;

    public Indexer() {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
    }

    /** Update the drive state of the indexer */
    public void drive(double speed) {
        drive.set(speed);
    }

}
