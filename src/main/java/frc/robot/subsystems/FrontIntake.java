package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Robot;

/** Front Intake Component */
public class FrontIntake {

    private static final int DRIVE_CAN = 10;

    private static final int ARTICULATE_CAN = 11;
    private static final double ARTICULATE_MAGNITUDE = 0.3;

    private final CANSparkMax drive, articulate;
    //private final ColorSensorV3 colorSensor;
    private RelativeEncoder encoder;

    private final Robot robot;

    public boolean shooterLock = false;

    // True/false is up/down state of intake
    private boolean articulateUp = true;

    private volatile boolean locked = false;

    public FrontIntake(Robot robot) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        articulate = new CANSparkMax(ARTICULATE_CAN, MotorType.kBrushless);
       // colorSensor = new ColorSensorV3(Port.kOnboard);
        this.robot = robot;
        encoder = articulate.getEncoder();


    }
    public void dropForShot() {
        articulate.set(0);
    }

    public void raise() {
        articulate.set(0);
    }

    /** Update the drive state of the front intake */
    public void update(boolean intake) {

      
        articulate.set(0);
    }

    public void rawArticulate(double percent) {
        articulate.set(percent);
    }

    public void driveIntakeOnly(double speed) {
        // articulate.set(0);
        drive.set(speed);
    }

    public void printColourVals() {
            System.out.println("Encoder Val: " + encoder.getPosition());
            System.out.println("Current: " + articulate.getOutputCurrent());
        
    }

}