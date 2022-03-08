package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;

/** Climber subsystem */
public class Climber {

    // UPDATE CAN IDs!
    private static final int LEFT = -1;

    private static final int RIGHT = -1;

    private final TalonFX left, right;

    public Climber() {
        this.left = new TalonFX(LEFT);
        this.right = new TalonFX(RIGHT);

        left.configForwardSoftLimitThreshold(2048 * 240);
        right.configForwardSoftLimitThreshold(2048 * 240);
        left.configReverseSoftLimitThreshold(-2048 * 240);
        right.configReverseSoftLimitThreshold(-2048 * 240);

        left.configForwardSoftLimitEnable(true);
        right.configForwardSoftLimitEnable(true);
        left.configReverseSoftLimitEnable(true);
        right.configReverseSoftLimitEnable(true);

        left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35.0, 50.0, 0.5));
        right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35.0, 50.0, 0.5));

        left.setInverted(true); // Might have to be switched to right at operators discression
    }

    double lastTime = 0;
    double lastLeftErr = 0;
    double lastRightErr = 0;
    double leftIntegrator = 0;
    double rightIntegrator = 0;

    /**
     * Update the desired state of the climber. Running on positional PID
     * @param desiredPosition the position, in degrees, relative to ..... (tbd)
     * @param isLoaded enter true if the climber will be lifting the robot in this movement
     */
    public void update(double desiredPosition, boolean isLoaded) {

        double kP_left = isLoaded ? 0.0 : 0.5; //EVERYTHING NEEDS TO BE TUNED
        double kI_left = isLoaded ? 0.0 : 0.0;
        double kD_left = isLoaded ? 0.0 : 0.0;

        double kP_right = isLoaded ? 0.0 : 0.5;
        double kI_right = isLoaded ? 0.0 : 0.0;
        double kD_right = isLoaded ? 0.0 : 0.0;

        double leftErr = left.getSelectedSensorPosition() / 240.0 - desiredPosition;
        double rightErr = right.getSelectedSensorPosition() / 240.0 - desiredPosition;

        double deltaT = Timer.getFPGATimestamp() - lastTime;

        leftIntegrator += leftErr;
        rightIntegrator += rightErr;
        
        double leftDer = (leftErr - lastLeftErr) / (deltaT);
        double rightDer = (rightErr - lastRightErr) / (deltaT);

        double leftOutput = (kP_left * leftErr + kI_left * leftIntegrator + kD_left * leftDer) / left.getBusVoltage();
        double rightOutput = kP_right * rightErr + kI_right * rightIntegrator + kD_right * rightDer / right.getBusVoltage();

        left.set(ControlMode.PercentOutput, leftOutput);
        right.set(ControlMode.PercentOutput, rightOutput);

        lastTime = Timer.getFPGATimestamp();
        lastLeftErr = left.getSelectedSensorPosition() - desiredPosition;
        lastRightErr = right.getSelectedSensorPosition() - desiredPosition;
    } 

    /**
     * For manual early testing, set output to the motors, prints how far it thinks the arm has traveled in degs for debugging
     * @param rawInput Percent, -1 to 1
     */
    public void test(double rawInput) {
        left.set(ControlMode.PercentOutput, rawInput);
        right.set(ControlMode.PercentOutput, rawInput);

        System.out.println("Left Reading: " + (left.getSelectedSensorPosition() / 240.0) + " degrees");
        System.out.println("Right Reading: " + (right.getSelectedSensorPosition() / 240.0) + " degrees");

        //System.out.println("Left Current: " + left.getStatorCurrent());
        //System.out.println("Right Current: " + right.getStatorCurrent());

        //System.out.println("Left Voltage: " + left.getBusVoltage());
        //System.out.println("Right Voltage: " + right.getBusVoltage());

        //System.out.println("Raw Left Enc: " + left.getSelectedSensorPosition());
        //System.out.println("Raw Right Enc: " + right.getSelectedSensorPosition());
    }

}


