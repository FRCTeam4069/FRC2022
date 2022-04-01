package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Robot;

/** Front Intake Component */
public class FrontIntake {

    private static final int DRIVE_CAN = 10;

    private static final int ARTICULATE_CAN = 11;
    private static final double ARTICULATE_MAGNITUDE = 0.3;

    private final CANSparkMax drive, articulate;
    //private final ColorSensorV3 colorSensor;
    private Encoder encoder;

    private final Robot robot;

    public boolean shooterLock = false;

    private final double bottomPos = 1040;
    double down_kP = 0.005;
    double up_kP = 0.008;

    // True/false is up/down state of intake
    private boolean articulateUp = true;

    private volatile boolean locked = false;

    public FrontIntake(Robot robot) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        articulate = new CANSparkMax(ARTICULATE_CAN, MotorType.kBrushless);
       // colorSensor = new ColorSensorV3(Port.kOnboard);
        this.robot = robot;
        encoder = new Encoder(8, 9, true, EncodingType.k1X);
    }
    public void dropForShot() {
        double error = bottomPos - encoder.getDistance();

        if(Math.abs(error) < 100) {
            articulate.set(0);
            return;
        }
        double output = down_kP * error;
        double sign = Math.signum(output);
        if(Math.abs(output) > 0.25) output = 0.25 * sign; 
        articulate.set(output);
    }

    public void raise() {
        double error = 0 - encoder.getDistance();
        if(Math.abs(error) < 100) {
            articulate.set(0);
            return;
        } 

        double output = up_kP * error;
        articulate.set(output);
    }


    public void rawArticulate(double percent) {
        articulate.set(percent);
    }

    public void driveIntakeOnly(double speed) {
        // articulate.set(0);
        drive.set(speed);
    }

    public void printColourVals() {
            System.out.println("Encoder Val: " + encoder.getDistance());
            System.out.println("Current: " + articulate.getOutputCurrent());
        
    }

}