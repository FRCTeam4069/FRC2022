package frc.robot.components;

import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

public class FrontIntake implements RobotComponent {

    private final Robot robot;

    CANSparkMax drive;
    CANSparkMax articulate;
    RelativeEncoder encoder;

    double kP_articulate = 0.0;

    public FrontIntake(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {
        drive = new CANSparkMax(Constants.FI_NEO_DRIVE, MotorType.kBrushless);
        articulate = new CANSparkMax(Constants.FI_NEO_ARTICULATE, MotorType.kBrushless);
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

        double error = encoderPos - encoder.getPosition();

        articulate.set(error * kP_articulate);

        //Likely need to add D or Feedforward in order to compensate for gravity
    }

    @Override
    public void loop() {

    }

    @Override
    public void shutdown() {
        drive.close();
        articulate.close();
    }

}