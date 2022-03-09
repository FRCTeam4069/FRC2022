package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
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
    private final ColorSensorV3 colorSensor;

    private final Robot robot;

    // True/false is up/down state of intake
    private boolean articulateUp = true;

    private volatile boolean locked = false;

    public FrontIntake(Robot robot) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        articulate = new CANSparkMax(ARTICULATE_CAN, MotorType.kBrushless);
        articulateEncoder = articulate.getEncoder();
        colorSensor = new ColorSensorV3(Port.kOnboard);
        this.robot = robot;
    }

    /** Update the drive state of the front intake */
    public void drive(double speed) {
        drive.set(speed);
    }

    /** Raise/lower front intake */
    public void articulate() {
        if (locked)
            return;
        locked = true;

        articulateUp = getPosition();
        // Flip stored state
        articulateUp = !articulateUp;
        // Reset
        articulateEncoder.setPosition(0);
        // Run thread
        robot.getScheduler().schedule((RobotAsyncTask) this::articulateFunc);
    }

    // Reset stored articulate position
    private boolean getPosition() {
        return false;
    }

    // Async function to check for articulation stop point
    private void articulateFunc() {
        // Starts in desired direction
        if (articulateUp)
            articulate.set(ARTICULATE_MAGNITUDE);
        else
            articulate.set(-ARTICULATE_MAGNITUDE);

        while (!(articulateEncoder.getPosition() < 0.5 && articulateUp)
                && !(Math.abs(articulateEncoder.getPosition() - 50) < 1.0 && !articulateUp));

        // Stops
        articulate.set(0);

        locked = false;
    }

    public void rawArticulate(double percent) {
        if (Math.abs(percent) > 0.25)
            articulate.set(percent);
        else
            articulate.set(0);
    }

}