package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Scheduler.RobotRepeatingTask;

/** Controls handlingß */
public class Controls {

    // IDs
    private static final int GP_1 = 0, GP_2 = 1;

    private static final double FRONT_INTAKE_DEADBAND = 0.05;
    private static final double INDEXER_DEADBAND = 0.2;

    // Cooldowns (ms)
    private static final int GEAR_CHANGE_CD = 1000; // 1s
    private static final int ARTICULATE_CD = 2500; // 2.5s

    // Controllers
    private XboxController controller1, controller2;

    // Call timestamps
    private long lastGearChange = 0;
    private long lastArticulate = 0;

    /**
     * Requires robot dependancy
     * 
     * @param robot Robot instance
     */
    public Controls(Robot robot) {
        controller1 = new XboxController(GP_1);
        controller2 = new XboxController(GP_2);

        robot.getScheduler().schedule(new RobotRepeatingTask() {
            @Override
            public void run() {
                // CONTROLS MAPPING
                switch (robot.getMode()) {
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
                         * 
                         * DRIVER 1:
                         * L Trigger - Drivetrain Forward
                         * R Trigger - Drivetrain Backward
                         * R Bumper - Drivetrain Change Gears
                         * L X Joystick - Drivetrain Turn
                         * 
                         * DRIVER 2:
                         * L Trigger - Front Intake Out
                         * R Trigger - Front Intake In
                         * L Bumper - Rear Intake Out
                         * R Bumper - Rear Intake In
                         * L +Y Joystick - Indexer Up
                         * L -Y Joystick - Indexer Down
                         * A - Shoter Auto Aim For High
                         * B - Close Low Goal
                         * X - Safezone High Goal
                         * Y - Close High Goal
                         * Start - Front Intake Articulate
                         */

                        // Drive
                        robot.getDriveTrain().arcadeDrive(
                                getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(),
                                getGamepad1().getLeftX());

                        // Change gear w/ cooldown
                        if (getGamepad1().getRightBumper()
                                && lastGearChange + GEAR_CHANGE_CD > System.currentTimeMillis()) {
                            robot.getDriveTrain().changeGear();
                            lastGearChange = System.currentTimeMillis();
                        }

                        // Front Intake
                        robot.getFrontIntake().drive(getGamepad2().getLeftTriggerAxis() > FRONT_INTAKE_DEADBAND
                                || getGamepad2().getRightTriggerAxis() > FRONT_INTAKE_DEADBAND,
                                getGamepad2().getLeftTriggerAxis() > FRONT_INTAKE_DEADBAND);
                        
                        // Front Intake Articulate
                        if (getGamepad2().getStartButton()
                                && lastArticulate + ARTICULATE_CD > System.currentTimeMillis()) {
                            robot.getFrontIntake().articulate();
                            lastArticulate = System.currentTimeMillis();
                        }

                        // Rear Intake
                        robot.getRearIntake().drive(getGamepad1().getLeftBumper() || getGamepad1().getRightBumper(),
                                getGamepad1().getLeftBumper());
                        
                        // Indexer
                        robot.getIndexer().drive(getGamepad2().getLeftY() > INDEXER_DEADBAND
                                || getGamepad2().getLeftY() < -INDEXER_DEADBAND,
                                getGamepad2().getLeftTriggerAxis() < -INDEXER_DEADBAND);
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
                        if (getGamepad1().getStartButton())
                            robot.getFlywheel().update(0, 1100);
                        else if (getGamepad1().getBackButton())
                            robot.getFlywheel().update(0, 900);
                        else
                            robot.getFlywheel().updatePercentage(0, 0);

                        // Indexer
                        // #drive(enabled = a or b is pressed, reversed = b is pressed)
                        robot.getIndexer().drive(getGamepad1().getAButton() || getGamepad1().getBButton(),
                                getGamepad1().getBButton());

                        // Front intake drive
                        // #drive(enabled = left bumper or right bumper is pressed, reversed = left
                        // bumper is pressed)
                        robot.getFrontIntake().drive(getGamepad1().getLeftBumper() || getGamepad1().getRightBumper(),
                                getGamepad1().getLeftBumper());

                        // Front intake articulatess

                        // Rear intake drive
                        // #drive(enabled = x or y is pressed, reversed = x is pressed)
                        robot.getRearIntake().drive(getGamepad1().getXButton() || getGamepad1().getYButton(),
                                getGamepad1().getXButton());

                        // Drivetrain
                        // #arcadeDrive(speed = right trigger - left trigger, turn = left joystick x
                        // axis)
                        robot.getDriveTrain().arcadeDrive(
                                getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(),
                                getGamepad1().getLeftX());

                        break;
                    default:
                        break;
                }

            }
        });
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
