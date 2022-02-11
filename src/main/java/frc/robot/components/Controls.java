package frc.robot.components;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class Controls implements RobotComponent {

    private final Robot robot;

    // Controllers
	private XboxController controller1;
	private XboxController controller2;

    /**
     * Requires robot dependancy
     * @param robot Robot instance
     */
    public Controls(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {
        controller1 = new XboxController(GP_1);
        controller2 = new XboxController(GP_2);
        return this;
    }

    @Override
    public void loop() {
        // CONTROLS MAPPING
        switch(robot.getMode()) {
            case AUTO:
                break;
            case DISABLED:
                break;
            case TELEOP:
                break;
            case TEST:
                break;
            default:
                break;
            
        }
    }

    public XboxController getGamepad1() {
        return controller1;
    }

    public XboxController getGamepad2() {
        return controller2;
    }
    
}
