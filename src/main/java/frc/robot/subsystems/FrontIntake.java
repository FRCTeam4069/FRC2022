package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.Scheduler.RobotAsyncTask;

/** Front Intake Component */
public class FrontIntake {

    private static final int DRIVE_CAN = 10;

    private static final int ARTICULATE_CAN = 11;
    private static final double ARTICULATE_MAGNITUDE = 0.3;
    private static final double ARTICULATE_POSITION = 50.0; // |pos|

    private final CANSparkMax drive, articulate;
    private final RelativeEncoder articulateEncoder;

    private final Robot robot;

    // True/false is up/down state of intake
    private boolean articulateUp = true;

    public FrontIntake(Robot robot) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        articulate = new CANSparkMax(ARTICULATE_CAN, MotorType.kBrushless);
        articulateEncoder = articulate.getEncoder();
        this.robot = robot;
    }

    /** Update the drive state of the front intake */
    public void drive(double speed) {
        drive.set(speed);
    }

    /** Raise/lower front intake */
    public void articulate() {
        // Flip stored state
        articulateUp = !articulateUp;
        // Reset
        articulateEncoder.setPosition(0);
        // Run thread
        robot.getScheduler().schedule((RobotAsyncTask) this::articulateFunc);
    }

    // Async function to check for articulation stop point
    private void articulateFunc() {
        // Starts in desired direction
        if (articulateUp) articulate.set(ARTICULATE_MAGNITUDE);
        else articulate.set(-ARTICULATE_MAGNITUDE);

        // Locks async thread until desired pos is reached
        while ((!articulateUp && articulateEncoder.getPosition() > -ARTICULATE_POSITION) ||
                (articulateUp && articulateEncoder.getPosition() < ARTICULATE_POSITION));

        // Stops
        articulate.set(0);
    }

    public void rawArticulate(double percent) {
        if(Math.abs(percent) > 0.25) articulate.set(percent);
        else articulate.set(0);
    }
    
}