package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.Robot;

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
    
    // Misc constants
    private static final double TIME_PERIOD = 0.02;
    private static final double FILTER_TIME_CONSTANT = 0.1;

    // Drivetrain hardware, etc
    private final TalonFX leftMaster, leftSlave, rightMaster, rightSlave;
    private final Encoder leftEncoder, rightEncoder;
    private final PIDController leftPid, rightPid;
    private final DoubleSolenoid shifter;
    private final LinearFilter leftFilter, rightFilter;

    private SimpleMotorFeedforward feedforward;
    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;

    private Robot robot;

    //Positional Stuff
    Pose2d currentPose;
    double lastLeft = 0;
    double lastRight = 0;

    private boolean highGear = false;

    public DriveTrain(Robot robot) {
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

        leftPid = new PIDController(LEFT_P, LEFT_I, LEFT_D, TIME_PERIOD);
        rightPid = new PIDController(RIGHT_P, RIGHT_I, RIGHT_D, TIME_PERIOD);

        shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, SHIFTER_FWD, SHIFTER_BCK);

        leftFilter = LinearFilter.singlePoleIIR(FILTER_TIME_CONSTANT, TIME_PERIOD);
        rightFilter = LinearFilter.singlePoleIIR(FILTER_TIME_CONSTANT, TIME_PERIOD);

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        this.robot = robot;
        currentPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
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
        leftMaster.set(ControlMode.PercentOutput, leftFilter.calculate(left));
        rightMaster.set(ControlMode.PercentOutput, rightFilter.calculate(right));
    }

    /** Stops the drivetrain */
    public void stop() {
        setPower(0, 0);
    }

    /**
     * Robot arcade drive
     * 
     * @param speed Speed of robot
     * @param turn Turn amount
     */
    public void arcadeDrive(double speed, double turn) {
        WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, turn, false);
        setPower(speeds.left, speeds.right);
    }

    /**
     * Drives the robot based off encoder tick amounts
     * 
     * @param leftTicks Ticks to left side of drivetrain
     * @param rightTicks Ticks to right side of drivetrain
     */
    public void driveEncTick(double leftTicks, double rightTicks) {
        
    }

    
     /**
      * Updates the left and right output voltages to the drivetriain based on passed wheel speeds
      * @param leftVel the velocity of the left side of the drivetrain in meters per second
      * @param rightVel the velocity of the right side of the drivetrain in meters per second
      */
    public void updateDriveSpeeds(double leftVel, double rightVel) {
        
        double leftOutput = feedforward.calculate(leftVel) / leftMaster.getBusVoltage();
        double rightOutput = feedforward.calculate(rightVel) / rightMaster.getBusVoltage();

        leftMaster.set(ControlMode.PercentOutput, leftOutput);
        rightMaster.set(ControlMode.PercentOutput, rightOutput);
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

    /** Re-zeroes */
    public void resetPos() {
        rightEncoder.reset();
        leftEncoder.reset();

        lastLeft = 0;
        lastRight = 0;
        currentPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    }

    /** Updates the active position of the bot */
    public void updatePos() {
        double currLeft = leftEncoder.getDistance();
        double currRight = rightEncoder.getDistance();

        double deltaLeft = currLeft - lastLeft;
        double deltaRight = currRight - lastRight;

        double heading = robot.getGyroscope().getFusedHeading();


        double distance = (deltaLeft + deltaRight) / 2.0;

        var translation = new Translation2d(distance * Math.cos(Math.toRadians(heading)), distance * Math.sin(Math.toRadians(heading)));
        var rotation = new Rotation2d(Math.toRadians(heading));

        currentPose = currentPose.transformBy(new Transform2d(translation, rotation));

        lastRight = leftEncoder.getDistance();
        lastLeft = rightEncoder.getDistance();
    }

    /** Get Pose */
    public Pose2d getPose() {
        return currentPose;
    }
}
