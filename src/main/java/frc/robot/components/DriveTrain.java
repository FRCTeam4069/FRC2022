package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DriveTrain implements RobotComponent {
    
    private Robot robot;

    private CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
    private Encoder leftEncoder, rightEncoder;
    private boolean highGear;

    private DoubleSolenoid shifter;

    /*
    * Interface methods
    */

    @Override
    public RobotComponent init(Robot robot) {
        // Dependency Injection
        this.robot = robot;

        // Motors (src: FRCTeam4069/Preseason2022/src/main/java/frc/robot/subsystems/Drivetrain.java#<init>)
        leftMaster = new CANSparkMax(DT_LEFT_MASTER, MotorType.kBrushless);
        leftSlave = new CANSparkMax(DT_LEFT_SLAVE, MotorType.kBrushless);
        rightMaster = new CANSparkMax(DT_RIGHT_MASTER, MotorType.kBrushless);
        rightSlave = new CANSparkMax(DT_RIGHT_SLAVE, MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftEncoder = new Encoder(DT_LEFT_MASTER_ENC, DT_LEFT_SLAVE_ENC, true, EncodingType.k1X);
        rightEncoder = new Encoder(DT_RIGHT_MASTER_ENC, DT_RIGHT_SLAVE_ENC, false, EncodingType.k1X);

        leftEncoder.reset();
        rightEncoder.reset();

        leftEncoder.setDistancePerPulse(1);
        rightEncoder.setDistancePerPulse(1);

        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, DT_SHIFTER_FWD, DT_SHIFTER_BCK);

        highGear = false;
        
        return this;
    }


    @Override
    public void shutdown() {
        // Closes motors (JNI)
        leftMaster.close();
        leftSlave.close();
        rightMaster.close();
        rightSlave.close();
    }
    

    /*
    * Basic movement
    */
    
    /**
     * Stops the drivetrain
     */
    public void stop() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();
    }


    /*
    * Gear shifting
    */

    /**
     * Sets the gear to high or low
     * @param highGear True if high gear preferred
     */
    public void setGear(boolean highGear) {
        // Change gear if values differ
        if (highGear != this.highGear) changeGear();        
    }


    /**
     * Inverts the gear state
     * High > Low, Low > High
     */
    public void changeGear() {
        // Flip gear state
        highGear = !highGear;

        if (highGear) shifter.set(Value.kForward);
        else shifter.set(Value.kReverse);
    }

}
