package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/** Climber subsystem */
public class Climber {

    private static final int LEFT = 15;
    private static final int RIGHT = 16;

    private final int LEFT_LONG_FIRE = 13;
    private final int LEFT_LONG_RETRACT = 2;

    private final int RIGHT_LONG_FIRE = 12;
    private final int RIGHT_LONG_RETRACT = 3;

    private final int LEFT_SHORT_FIRE = 11;
    private final int LEFT_SHORT_RETRACT = 4;

    private final int RIGHT_SHORT_FIRE = 10;
    private final int RIGHT_SHORT_RETRACT = 5;

    private DoubleSolenoid rightLong, leftLong, rightShort, leftShort;

    private final TalonFX left, right;

    public Climber() {
        this.left = new TalonFX(LEFT);
        this.right = new TalonFX(RIGHT);

        // left.configForwardSoftLimitThreshold(2048 * 240);
        // right.configForwardSoftLimitThreshold(2048 * 240);
        // left.configReverseSoftLimitThreshold(-2048 * 240);
        // right.configReverseSoftLimitThreshold(-2048 * 240);

        left.configForwardSoftLimitEnable(false);
        right.configForwardSoftLimitEnable(false);
        left.configReverseSoftLimitEnable(false);
        right.configReverseSoftLimitEnable(false);

        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);

        left.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35.0, 50.0, 0.5));
        right.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35.0, 50.0, 0.5));

        left.setInverted(true);
        left.follow(right);

        // leftLong = new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_LONG_FIRE, LEFT_LONG_RETRACT);
        // rightLong = new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_LONG_FIRE, RIGHT_LONG_RETRACT);
        // leftShort = new DoubleSolenoid(PneumaticsModuleType.REVPH, LEFT_SHORT_FIRE, LEFT_SHORT_FIRE);
        // rightShort = new DoubleSolenoid(PneumaticsModuleType.REVPH, RIGHT_SHORT_FIRE, RIGHT_SHORT_RETRACT);
    }

    double lastTime = 0;
    double lastErr = 0;
    double integrator = 0;


    /**
     * Update the desired state of the climber. Running on positional PID
     * @param desiredPosition the position, in degrees, relative to ..... (tbd)
     * @param isLoaded enter true if the climber will be lifting the robot in this movement
     */
    public void update(double desiredPosition, boolean isLoaded) {

        double kP = isLoaded ? 0.0 : 0.8; 
        double kI = isLoaded ? 0.0 : 0.0;
        double kD = isLoaded ? 0.0 : 0.0;

        double Err = desiredPosition - ((right.getSelectedSensorPosition() / (320.0 * 2048)) * 360);

        if(Math.abs(Err) < 1) {
            right.set(ControlMode.PercentOutput, 0);
            return;
        } 

        double deltaT = Timer.getFPGATimestamp() - lastTime;

        integrator += Err;
        
        double Der = (Err - lastErr) / (deltaT);

        double output = (kP * Err + kI * integrator + kD * Der) / right.getBusVoltage();

        right.set(ControlMode.PercentOutput, output);

        System.out.println("Error: " + Err);

        lastTime = Timer.getFPGATimestamp();
        lastErr = right.getSelectedSensorPosition() - desiredPosition;
    } 

    public void fireLong() {
        leftLong.set(Value.kForward);
        rightLong.set(Value.kForward);
    }

    public void retractLong() {
        leftLong.set(Value.kReverse);
        rightLong.set(Value.kReverse);
    }

    public void fireShort() {
        leftShort.set(Value.kForward);
        rightShort.set(Value.kForward);
    }

    public void retractShort() {
        leftShort.set(Value.kReverse);
        rightShort.set(Value.kReverse);
    }

    /**
     * For manual early testing, set output to the motors, prints how far it thinks the arm has traveled in degs for debugging
     * @param rawInput Percent, -1 to 1
     */
    public void test(double rawInput) {
        if(Math.abs(rawInput) > 0.15) {
            right.set(ControlMode.PercentOutput, rawInput);
        }
        else {
            right.set(ControlMode.PercentOutput, 0);
        }

        // System.out.println("Left Reading: " + (left.getSelectedSensorPosition() / (320 * 2048)) * 360);
        System.out.println("Right Reading: " + (right.getSelectedSensorPosition() / (320 * 2048)) * 360);

        // System.out.println("Left Current: " + left.getStatorCurrent());
        // System.out.println("Right Current: " + right.getStatorCurrent());

        //System.out.println("Left Voltage: " + left.getBusVoltage());
        //System.out.println("Right Voltage: " + right.getBusVoltage());

        //System.out.println("Raw Left Enc: " + left.getSelectedSensorPosition());
        //System.out.println("Raw Right Enc: " + right.getSelectedSensorPosition());
    }

    public void resetEncoders() {
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);
    }

    public void testDual(double rawInputLeft, double rawInputRight) {
        if(Math.abs(rawInputLeft) > 0.15) {
            left.set(ControlMode.PercentOutput, rawInputLeft);
        }
        else {
            left.set(ControlMode.PercentOutput, 0);
        }

        if(Math.abs(rawInputRight) > 0.15) {
            right.set(ControlMode.PercentOutput, rawInputRight);
        }
        else {
            right.set(ControlMode.PercentOutput, 0);
        }

        

        System.out.println("Left Reading: " + ((left.getSelectedSensorPosition() / (2048 * 240.0)) * 360) + " degrees");
        System.out.println("Right Reading: " + ((right.getSelectedSensorPosition() / (2048 * 240.0)) * 360) + " degrees");
        System.out.println("Left Current: " + left.getStatorCurrent());
        System.out.println("Right Current: " + right.getStatorCurrent());

        //System.out.println("Left Voltage: " + left.getBusVoltage());
        //System.out.println("Right Voltage: " + right.getBusVoltage());

        //System.out.println("Raw Left Enc: " + left.getSelectedSensorPosition());
        //System.out.println("Raw Right Enc: " + right.getSelectedSensorPosition());
    }
    public double getCurrent() {
        return right.getStatorCurrent();
    }


}


