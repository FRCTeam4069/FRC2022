package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DriveTrain implements RobotComponent {
    
    private boolean highGear = false;

    private final Robot robot;

    private final TalonFX leftMaster = new TalonFX(DT_LEFT_MASTER),
        leftSlave = new TalonFX(DT_LEFT_SLAVE), 
        rightMaster = new TalonFX(DT_RIGHT_MASTER), 
        rightSlave = new TalonFX(DT_RIGHT_SLAVE);

    private final Encoder leftEncoder = new Encoder(DT_LEFT_MASTER_ENC, DT_LEFT_SLAVE_ENC, true, EncodingType.k1X),
        rightEncoder = new Encoder(DT_RIGHT_MASTER_ENC, DT_RIGHT_SLAVE_ENC, false, EncodingType.k1X);
    

    private final PIDController leftPid = new PIDController(DT_LEFT_P, DT_LEFT_I, DT_LEFT_D), 
            rightPid = new PIDController(DT_RIGHT_P, DT_RIGHT_I, DT_RIGHT_D);

    private final DoubleSolenoid shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, DT_SHIFTER_FWD, DT_SHIFTER_BCK);

    private final PigeonIMU gyro;

    /*
    * Interface methods
    */

    /**
     * @param robot Robot instance
     * @param gyro Shared gyro instance
     */
    public DriveTrain(Robot robot, PigeonIMU gyro) {
        this.robot = robot;
        this.gyro = gyro;
    }

    @Override
    public RobotComponent init() {
        // Motors
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);

        // Encoder
        leftEncoder.reset();
        rightEncoder.reset();

        leftEncoder.setDistancePerPulse(1);
        rightEncoder.setDistancePerPulse(1);
        
        return this;
    }

    @Override
    public void loop() {
        
    }

    @Override
    public void shutdown() {
        
    }
    

    /*
    * Readings 
    */

    public double getAvgVelocity() {
        return leftEncoder.getRate() + rightEncoder.getRate() / 2;
    }

    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    public double getRightVelocity() {
        return rightEncoder.getRate();
    }

    /*
    * Basic movement
    */
    
    /**
     * Sets the power of each side of the drivetrain (-1 to 1)
     * @param left Left side power
     * @param right Right side power
     */
    public void setPower(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    /**
     * Stops the drivetrain
     */
    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
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
