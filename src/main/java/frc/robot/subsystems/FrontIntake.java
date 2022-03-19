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

    private final CANSparkMax drive, articulate;
    private final ColorSensorV3 colorSensor;

    private final Robot robot;

    // True/false is up/down state of intake
    private boolean articulateUp = true;

    private volatile boolean locked = false;

    public FrontIntake(Robot robot) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        articulate = new CANSparkMax(ARTICULATE_CAN, MotorType.kBrushless);
        colorSensor = new ColorSensorV3(Port.kOnboard);
        this.robot = robot;
    }

    /** Update the drive state of the front intake */
    public void update(boolean intake) {

        if(intake) {
            drive.set(1);

            if(colorSensor.getProximity() > 500) articulate.set(-0.5);
            else articulate.set(0);
        }

        else {
            drive.set(0);

            if(colorSensor.getProximity() < 500) articulate.set(0.3);
            else articulate.set(0);
        }
    }

    public void rawArticulate(double percent) {
        System.out.println("Percent: " + percent);
        if (Math.abs(percent) > 0.25 && Math.abs(percent) < 0.5)
            articulate.set(percent);
        else if(Math.abs(percent) > 0.5)
            articulate.set(0.5 * Math.signum(percent));
        else
            articulate.set(0);
    }

    public void driveIntakeOnly(double speed) {
        articulate.set(0);
        drive.set(speed);
    }

    public void printColourVals() {
        if(colorSensor.isConnected()) {
            // System.out.println("Red: " + colorSensor.getRed());
            // System.out.println("Blue: " + colorSensor.getBlue());
            // System.out.println("Green: " + colorSensor.getGreen());
            // System.out.println("IR: " + colorSensor.getIR());
            System.out.println("Distance: " + colorSensor.getProximity());
        }
    }

}