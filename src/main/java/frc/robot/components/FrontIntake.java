package frc.robot.components;

import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

/**
 * Front intake component
 */
public class FrontIntake implements RobotComponent {

    private final Robot robot;

    CANSparkMax drive;
    CANSparkMax articulate;
    RelativeEncoder encoder;

    double kPArticulate = 0.0;

    public FrontIntake(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {
        drive = new CANSparkMax(FI_NEO_DRIVE, MotorType.kBrushless);
        //articulate = new CANSparkMax(FI_NEO_ARTICULATE, MotorType.kBrushless);
        encoder = articulate.getEncoder(Type.kHallSensor, 42);
        
        return this;
    }

    /**
     * Update desired articulation, driven percentage
     * @param drivenPercentage Between -1 and 1, percentage of driven power on intake/feed
     * @param encoderPos between 0 (fully retracted) and x (fully deployed)
     */
    public void update(double drivenPercentage, double encoderPos) {
        drive.set(drivenPercentage);

        //double error = encoderPos - encoder.getPosition();

        //articulate.set(error * kPArticulate);

        //Likely need to add D or Feedforward in order to compensate for gravity
    }

    public void updateRaw(double drivenPercentage, double articulatePercentage) {
        drive.set(drivenPercentage);
        //articulate.set(articulatePercentage);

        System.out.println("Articulated encoder Pos: " + encoder.getPosition());
    }

    @Override
    public void loop() {

    }

    @Override
    public void shutdown() {
        drive.close();
        //articulate.close();
    }

}