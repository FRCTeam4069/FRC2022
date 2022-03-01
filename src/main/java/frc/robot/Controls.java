package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Controls handlingß */
public class Controls {

    public static final int GP_1 = 0;
    public static final int GP_2 = 1;

    private final Robot robot;

    // Controllers
	private XboxController controller1;
	private XboxController controller2;

    /**
     * Requires robot dependancy
     * 
     * @param robot Robot instance
     */
    public Controls(Robot robot) {
        this.robot = robot;

        controller1 = new XboxController(GP_1);
        controller2 = new XboxController(GP_2);
    }

    /** Runs on loop method, checks for controls being pressed & processes them accordingly */
    public void parseControls() {
        // CONTROLS MAPPING
        switch(robot.getMode()) {
            case AUTO:

                /*
                 * AUTO MODE CONTROLS
                 */

                break;
            case DISABLED:

                /**
                 * MODE DISABLED CONTROLS
                 */

                break;
            case TELEOP:

                /**
                 * TELEOP MODE CONTROLS
                 */

                break;
            case TEST:

                /**
                 * TEST MODE CONTROLS
                 * 
                 * Start - Bottom wheel angular vel. of 1100
                 * Back - Bottom wheel angular vel. of 1100
                 * A - Indexer rotateOne
                 * B - Indexer rotateTwo
                 * Left Bumper - FI Forward
                 * Right Bumper - FI Backward
                 * Left Stick In - FI Articulate Forward
                 * Right Stick In - FI Articulate Backward
                 * X - RI Forward
                 * Y - RI Backward
                 * Right Trigger - Drivetrain forward
                 * Left Trigger - Drivetrain reverse
                 * Left Joystick X Axis - Drivetrain turn
                 * 
                 */

                // Flywheel
                if (getGamepad1().getStartButton()) robot.getFlywheel().update(0, 1100);
		        else if (getGamepad1().getBackButton()) robot.getFlywheel().update(0, 900);
		        else robot.getFlywheel().updatePercentage(0, 0);

                // Indexer 
                // #drive(enabled = a or b is pressed, reversed = b is pressed)
		        robot.getIndexer().drive(getGamepad1().getAButton() || getGamepad1().getBButton(), getGamepad1().getBButton());

                // Front intake drive
                // #drive(enabled = left bumper or right bumper is pressed, reversed = left bumper is pressed)
		        robot.getRearIntake().drive(getGamepad1().getLeftBumper() || getGamepad1().getRightBumper(), getGamepad1().getLeftBumper());

                // Rear intake articulate
		        // TODO

                // Rear intake drive
		        // #drive(enabled = x or y is pressed, reversed = x is pressed)
		        robot.getRearIntake().drive(getGamepad1().getXButton() || getGamepad1().getYButton(), getGamepad1().getXButton());

                // Drivetrain
                // #arcadeDrive(speed = right trigger - left trigger, turn = left joystick x axis)
                robot.getDriveTrain().arcadeDrive(getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(), getGamepad1().getLeftX());

                break;
            default:
                break;
        }
    }

    /** Gamepad for first driver */
    public XboxController getGamepad1() {
        return controller1;
    }

    /** Gamepad for second driver */
    public XboxController getGamepad2() {
        return controller2;
    }
    
}
