package frc.robot.subsystems;

import java.util.concurrent.ConcurrentSkipListSet;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/** Front Intake Component */
public class FrontIntake {

    double lastTime = 0;
    double lastPos = 0;

    private static final int DRIVE_CAN = 10;

    private static final int ARTICULATE_CAN = 11;
    private static final double ARTICULATE_MAGNITUDE = 0.3;

    private final double currentSpikeThreshold = 5;

    private final CANSparkMax drive, articulate;
    //private final ColorSensorV3 colorSensor;
    public Encoder encoder;

    private final Robot robot;

    public boolean shooterLock = false;

    private double upPosition = 0;
    private double downPosition = 1040;

    private boolean currentSpikeLast = false;
    private double currentSpikeStartTimer = 0;

    double down_kP = 0.005;
    double up_kP = 0.005;

    // True/false is up/down state of intake
    private boolean articulateUp = true;

    public void reset() {
        encoder.reset();
        upPosition = 0;
        downPosition= 1040;
    }

    private volatile boolean locked = false;

    public FrontIntake(Robot robot) {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
        articulate = new CANSparkMax(ARTICULATE_CAN, MotorType.kBrushless);
       // colorSensor = new ColorSensorV3(Port.kOnboard);
       
        this.robot = robot;
    }

    public double getVel() {
        double vel = (encoder.getDistance() - lastPos) / (Timer.getFPGATimestamp() - lastTime);
        lastPos = encoder.getDistance();
        lastTime = Timer.getFPGATimestamp();

        return vel;
    }


    public void dropForShot() {
        System.out.println("Desired Position: " + downPosition);
        System.out.println("ACtual Position: " + encoder.getDistance());
        System.out.println("Current: " + articulate.getOutputCurrent());
        double error = downPosition - encoder.getDistance();

        if(Math.abs(error) < 150) {
            articulate.set(0);
            return;
        }

        if(!currentSpikeLast && articulate.getOutputCurrent() > currentSpikeThreshold) {
            currentSpikeStartTimer = Timer.getFPGATimestamp();
            currentSpikeLast = true;
        }
        else if(currentSpikeLast && articulate.getOutputCurrent() > currentSpikeThreshold) {
            if(Timer.getFPGATimestamp() > currentSpikeStartTimer + 0.5) {
                encoder.reset();
                upPosition = -1040;
                downPosition = 0;
            }
        }
        else {
            currentSpikeStartTimer = 0;
            currentSpikeLast = false;
        }

        double output = down_kP * error;
        double sign = Math.signum(output);
        if(Math.abs(output) > 0.35) output = 0.35 * sign; 
        articulate.set(output);
    }



    public void raise() {
        double error = upPosition - encoder.getDistance();

        System.out.println("Desired Position: " + upPosition);
        System.out.println("ACtual Position: " + encoder.getDistance());
        System.out.println("Current: " + articulate.getOutputCurrent());
        if(Math.abs(error) < 150) {
            articulate.set(0);
            return;
        } 

        if(!currentSpikeLast && articulate.getOutputCurrent() > currentSpikeThreshold) {
            currentSpikeStartTimer = Timer.getFPGATimestamp();
            currentSpikeLast = true;
        }
        else if(currentSpikeLast && articulate.getOutputCurrent() > currentSpikeThreshold) {
            if(Timer.getFPGATimestamp() > currentSpikeStartTimer + 0.5) {
                encoder.reset();
                upPosition = 0;
                downPosition = 1040;
            }
        }
        else {
            currentSpikeStartTimer = 0;
            currentSpikeLast = false;
        }

        double output = up_kP * error;
        double sign = Math.signum(output);
        if(Math.abs(output) > 0.35) output = 0.35 * sign; 
        articulate.set(output);
    }


    public void rawArticulate(double percent) {
        articulate.set(percent);

    if(Math.abs(percent) > 0.1) System.out.println("Current: " + articulate.getOutputCurrent());
    }

    public void driveIntakeOnly(double speed) {
        // articulate.set(0);
        drive.set(speed);
    }

    public void printColourVals() {
            System.out.println("Encoder Val: " + encoder.getDistance());
            System.out.println("Current: " + articulate.getOutputCurrent());
        
    }
    public void printNumbers(){
        SmartDashboard.putNumber("Current Pos", ARTICULATE_CAN);
        SmartDashboard.putNumber("Current", ARTICULATE_CAN);

    }

}