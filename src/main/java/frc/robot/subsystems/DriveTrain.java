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
    
    // Gear sensitivity, (2020 port)
    public static final double DT_HIGH_GEAR_SENSITIVITY = 0.35;
    public static final double DT_HIGH_GEAR_MOVING_SENSITIVITY = 0.45;
    public static final double DT_LOW_GEAR_SENSITIVITY = 0.625;

    // PID
    public static final double DT_LEFT_P = 0.6;
    public static final double DT_LEFT_I = 0.0;
    public static final double DT_LEFT_D = 0.0;

    public static final double DT_RIGHT_P = 0.6;
    public static final double DT_RIGHT_I = 0.0;
    public static final double DT_RIGHT_D = 0.0;

    // Hardware IDs
    public static final int DT_LEFT_MASTER = 4;
    public static final int DT_LEFT_SLAVE = 5;
    public static final int DT_LEFT_MASTER_ENC = 0;
    public static final int DT_LEFT_SLAVE_ENC = 1;

    public static final int DT_RIGHT_MASTER = 6;
    public static final int DT_RIGHT_SLAVE = 7;
    public static final int DT_RIGHT_MASTER_ENC = 2;
    public static final int DT_RIGHT_SLAVE_ENC = 3;

    public static final int DT_SHIFTER_FWD = 0; // UPDATE
    public static final int DT_SHIFTER_BCK = 15; // UPDATE

    // Deadband for controller stick drift
    public static final double DT_ARCADE_DEADBAND = 0.12;

    // Drivetrain hardware, etc
    private final TalonFX leftMaster, leftSlave, rightMaster, rightSlave;
    private final Encoder leftEncoder, rightEncoder;
    private final PIDController leftPid, rightPid;
    private final DoubleSolenoid shifter;

    private boolean highGear = false;

    public DriveTrain() {
        leftMaster = new TalonFX(DT_LEFT_MASTER);
        leftSlave = new TalonFX(DT_LEFT_SLAVE);
        rightMaster = new TalonFX(DT_RIGHT_MASTER);
        rightSlave = new TalonFX(DT_RIGHT_SLAVE);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.setInverted(true);
        rightSlave.setInverted(true);

        leftEncoder = new Encoder(DT_LEFT_MASTER_ENC, DT_LEFT_SLAVE_ENC, true, EncodingType.k1X);
        rightEncoder = new Encoder(DT_RIGHT_MASTER_ENC, DT_RIGHT_SLAVE_ENC, false, EncodingType.k1X);
        // leftPid = new PIDController(DT_LEFT_P, DT_LEFT_I, DT_LEFT_D);
        // rightPid = new PIDController(DT_RIGHT_P, DT_RIGHT_I, DT_RIGHT_D);
        
        // TO BE REMOVED AND REPLACED WITH ABOVE
        leftPid = null;
        rightPid = null;
        
        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, DT_SHIFTER_FWD, DT_SHIFTER_BCK);
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
        // SRC: edu.wpi.first.wpilibj.drive.DifferentialDrive
        // Cutoff below certain vals
        speed = MathUtil.applyDeadband(speed, DT_ARCADE_DEADBAND);
        turn = MathUtil.applyDeadband(turn, DT_ARCADE_DEADBAND);
        
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

    /**
     * Inverts the gear state
     * <p>
     * High > Low, Low > High
     */
    public void changeGear() {
        // Flip gear state
        highGear = !highGear;

        if (highGear) shifter.set(Value.kForward);
        else shifter.set(Value.kReverse);
    }

}
