package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

/** Drivetrain Subsystem */
public class DriveTrain {

    // PID
    private static final double LEFT_P = 0.6;
    private static final double LEFT_I = 0.0;
    private static final double LEFT_D = 0.0;

    private static final double RIGHT_P = 0.6;
    private static final double RIGHT_I = 0.0;
    private static final double RIGHT_D = 0.0;

    // Hardware IDs
    private static final int LEFT_MASTER = 4;
    private static final int LEFT_SLAVE = 5;
    private static final int LEFT_ENC_A = 0;
    private static final int LEFT_ENC_B = 1;

    private static final int RIGHT_MASTER = 6;
    private static final int RIGHT_SLAVE = 7;
    private static final int RIGHT_ENC_A = 2;
    private static final int RIGHT_ENC_B = 3;

    private static final int SHIFTER_FWD = 0;
    private static final int SHIFTER_BCK = 15;

    // Deadband for controller stick drift
    private static final double ARCADE_DEADBAND = 0.12;

    // Drivetrain hardware, etc
    private final TalonFX leftMaster, leftSlave, rightMaster, rightSlave;
    private final Encoder leftEncoder, rightEncoder;
    private final PIDController leftPid, rightPid;
    private final DoubleSolenoid shifter;

    private boolean highGear = false;

    public DriveTrain() {
        leftMaster = new TalonFX(LEFT_MASTER);
        leftSlave = new TalonFX(LEFT_SLAVE);
        rightMaster = new TalonFX(RIGHT_MASTER);
        rightSlave = new TalonFX(RIGHT_SLAVE);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        leftEncoder = new Encoder(LEFT_ENC_A, LEFT_ENC_B, true, EncodingType.k1X);
        rightEncoder = new Encoder(RIGHT_ENC_A, RIGHT_ENC_B, false, EncodingType.k1X);
        leftPid = new PIDController(LEFT_P, LEFT_I, LEFT_D);
        rightPid = new PIDController(RIGHT_P, RIGHT_I, RIGHT_D);
        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, SHIFTER_FWD, SHIFTER_BCK);
    }

    /** Average rate between both encoders */
    public double getAvgVelocity() {
        return leftEncoder.getRate() + rightEncoder.getRate() / 2;
    }
    
    /** Rate of left encoder */
    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    /** Rate of right encoder */
    public double getRightVelocity() {
        return rightEncoder.getRate();
    }
    
    /**
     * Sets the power of each side of the drivetrain (-1 to 1)
     * 
     * @param left Left side power
     * @param right Right side power
     */
    public void setPower(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    /** Stops the drivetrain */
    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Robot arcade drive
     * 
     * @param speed Speed of robot
     * @param turn Turn amount
     */
    public void arcadeDrive(double speed, double turn) {
        // Current controllers have stick drift issues
        speed = MathUtil.applyDeadband(speed, ARCADE_DEADBAND);
        turn = MathUtil.applyDeadband(turn, ARCADE_DEADBAND);
        
        WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, turn, false);

        leftMaster.set(ControlMode.PercentOutput, speeds.left);
        rightMaster.set(ControlMode.PercentOutput, speeds.right);
    }

    /**
     * Sets the gear to high or low
     * 
     * @param highGear True if high gear preferred
     */
    public void setGear(boolean highGear) {
        // Change gear if values differ
        if (highGear != this.highGear) changeGear();        
    }

    /** Inverts the gear state */
    public void changeGear() {
        // Flip gear state
        highGear = !highGear;

        if (highGear) shifter.set(Value.kForward);
        else shifter.set(Value.kReverse);
    }

}
