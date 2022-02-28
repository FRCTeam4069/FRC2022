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

                if (getGamepad1().getStartButton()) robot.getFlywheel().update(0, 1100);
		        else if (getGamepad1().getBackButton()) robot.getFlywheel().update(0, 900);
		        else robot.getFlywheel().updatePercentage(0, 0);
		        robot.getIndexer().update(getGamepad1().getAButton(), getGamepad1().getBButton());

		        double fiDrivenPercentage = 0;
		        if (getGamepad1().getLeftBumper()) fiDrivenPercentage = 1;
		        else if(getGamepad1().getRightBumper()) fiDrivenPercentage = -1;

		        double fiArticulatePercentage = 0;
		        if (getGamepad1().getLeftStickButton()) fiArticulatePercentage = 0.75;
		        else if (getGamepad1().getRightStickButton()) fiArticulatePercentage = -0.75;		
		        robot.getFrontIntake().updateRaw(fiDrivenPercentage, fiArticulatePercentage);

		        double riDrivenPercentage = 0;
		        if (getGamepad1().getXButton()) riDrivenPercentage = 1;
	        	else if (getGamepad1().getYButton()) riDrivenPercentage = -1;
		        robot.getRearIntake().update(riDrivenPercentage);

                robot.getDriveTrain().arcadeDrive(getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(), getGamepad1().getLeftX());

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
