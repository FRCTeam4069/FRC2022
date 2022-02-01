package frc.robot.components;

import frc.robot.Robot;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

/**
 * Rear intake component
 */
public class RearIntake implements RobotComponent {

	/*
	Front and rear intake are functionally similar.

	Code 'borrowed' from the FrontIntake class.
	(Sorry to those assigned components, we needed this to prototype.)
	 */

	private final Robot robot;

	private CANSparkMax drive, articulate;
	private RelativeEncoder encoder;

	double kPArticulate = 0.0;

	/**
	 * @param robot Robot instance
	 */
	public RearIntake(Robot robot) {
		drive = new CANSparkMax(RI_NEO_DRIVE, MotorType.kBrushless);
        articulate = new CANSparkMax(RI_NEO_ARTICULATE, MotorType.kBrushless);
        encoder = articulate.getEncoder(Type.kHallSensor, 42);

		this.robot = robot;
	}

	// init function for the RearIntake
	@Override
	public RobotComponent init() {
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

        articulate.set(error * kPArticulate);

        //Likely need to add D or Feedforward in order to compensate for gravity
    }

	@Override
	public void loop() {
		
	}

	// Function to turn the component off
	@Override
	public void shutdown() {
		drive.close();
        articulate.close();
	}
}
