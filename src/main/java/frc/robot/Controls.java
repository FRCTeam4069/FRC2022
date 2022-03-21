package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Scheduler.RobotRepeatingTask;


/** Controls handlingß */
public class Controls {

    // IDs
    private static final int GP_1 = 0, GP_2 = 1;

    private static final double DRIVETRAIN_TRIGGER_DEADBAND = 0.05;
    private static final double DRIVETRAIN_STICK_DEADBAND = 0.2;
    private static final double INDEXER_DEADBAND = 0.2;

    // Cooldowns (ms)
    private static final int GEAR_CHANGE_CD = 1000; // 1s

    private final Robot robot;

    // Controllers
    private XboxController controller1, controller2;

    // Call timestamps
    private long lastGearChange = 0;

    /**
     * Requires robot dependancy
     * 
     * @param robot Robot instance
     */
    public Controls(Robot robot) {
        this.robot = robot;

        controller1 = new XboxController(GP_1);
        controller2 = new XboxController(GP_2);

        robot.getScheduler().schedule((RobotRepeatingTask) this::controlsCheck);
    }

    boolean intakeUp = true;
    boolean shooterIntakeLockout = false;
    private void controlsCheck() {
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
                 * L Trigger - Drivetrain Forward  *
                 * R Trigger - Drivetrain Backward  *
                 * R Bumper - Drivetrain Change Gears  *
                 * L X Joystick - Drivetrain Turn   *
                 * A - Shoter Auto Aim For High  *
                 * B - Close Low Goal *
                 * X - Safezone High Goal *
                 * Y - Close High Goal  *
                 * 
                 * DRIVER 2:
                 * L Trigger - Front Intake Out  *
                 * R Trigger - Front Intake In  *
                 * L Bumper - Rear Intake Out  *
                 * R Bumper - Rear Intake In  *
                 * L +Y Joystick - Indexer Up  *
                 * L -Y Joystick - Indexer Down  *
                 * 
                 */

                // Drive
                robot.getDriveTrain().arcadeDrive(
                        MathUtil.applyDeadband(getGamepad1().getRightTriggerAxis(), DRIVETRAIN_TRIGGER_DEADBAND)
                                - MathUtil.applyDeadband(getGamepad1().getLeftTriggerAxis(),
                                        DRIVETRAIN_TRIGGER_DEADBAND),
                        MathUtil.applyDeadband(getGamepad1().getLeftX(), DRIVETRAIN_STICK_DEADBAND));

                // Change gear w/ cooldown
                if (getGamepad1().getRightBumper()
                        && lastGearChange + GEAR_CHANGE_CD < System.currentTimeMillis()) {
                    robot.getDriveTrain().changeGear();
                    lastGearChange = System.currentTimeMillis();
                }

                // Front Intake
                if(getGamepad2().getRightTriggerAxis() > 0.5) robot.getFrontIntake().driveIntakeOnly(-1);
                else if(getGamepad2().getLeftTriggerAxis() > 0.5) robot.getFrontIntake().driveIntakeOnly(1);
                else robot.getFrontIntake().driveIntakeOnly(0);

                if(getGamepad2().getXButtonPressed() && !shooterIntakeLockout) intakeUp = !intakeUp;

                if(intakeUp) robot.getFrontIntake().raise();
                else robot.getFrontIntake().dropForShot();
                

                // Rear Intake
                robot.getRearIntake().drive(getGamepad2().getLeftBumper() || getGamepad2().getRightBumper(),
                        getGamepad2().getLeftBumper());

                // Indexer
                if(getGamepad2().getLeftBumper()) robot.getIndexer().drive(1);
                else if(getGamepad2().getLeftY() > 0.5) robot.getIndexer().drive(1);
                else if(getGamepad2().getLeftY() < -0.5) robot.getIndexer().drive(-1);
                else robot.getIndexer().drive(0);

                boolean startedShootingProcess = false;
                // Shooter
                if(getGamepad1().getAButton()) {

                    intakeUp = false;
                    shooterIntakeLockout = true;

                    if(!startedShootingProcess) {
                        startedShootingProcess = true;
                        robot.getDriveTrain().resetTurnError();
                        robot.getVision().enableLED();
                        robot.getDriveTrain().setGear(false);
                    }

                    if(!robot.getVision().hasTarget()) {
                        System.out.println("no target, assuming close shot");
                        robot.getDriveTrain().stop();
                        // robot.getFlywheel().update(1300, 410);
                        return;
                    }


                    if(Math.abs(robot.getDriveTrain().getTurnError()) > 3.0) {
                        robot.getDriveTrain().alignToGoal();
                    }
                    else {
                        robot.getDriveTrain().stop();
                    }

                    // robot.getFlywheel().updateDistance(robot.getVision().getDistance());
                }
                else {
                    startedShootingProcess = false;
                    robot.getVision().disableLED();
                    robot.getFlywheel().update(0, 0);
                    robot.getDriveTrain().alignFirstTime = true;
                    robot.getDriveTrain().endLockout();
                    robot.getFrontIntake().shooterLock = false;
                    shooterIntakeLockout = false;
                }

                //Chip shot - Speeds need testing and updating
                if(getGamepad1().getBButton()) robot.getFlywheel().update(0, 0);

                //Protected shot - Speeds need testing and updating
                if(getGamepad1().getXButton()) robot.getFlywheel().update(0, 0);


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

                if(getGamepad1().getAButton()) robot.getDriveTrain().resetPos();
                // robot.getDriveTrain().updatePos();
                // System.out.println("X: " + robot.getDriveTrain().getPose().getX());
                // System.out.println("Y: " + robot.getDriveTrain().getPose().getY());
                // System.out.println("Theta: " + robot.getDriveTrain().getPose().getRotation().getDegrees());

                robot.getClimber().test(getGamepad1().getRightY());



                robot.getDriveTrain().stop();
                //robot.getFrontIntake().driveIntakeOnly(0);
                // // // Flywheel - Close Shot
                // if (getGamepad1().getStartButton())
                //     robot.getFlywheel().update(800, 950);

                // //Distance shot
                // else if (getGamepad1().getBackButton())
                //     robot.getFlywheel().update(800, 900);
                // else
                //     robot.getFlywheel().updatePercentage(0, 0);

                // if(getGamepad1().getAButton()) robot.getIndexer().drive(1);
                // else if(getGamepad1().getBButton()) robot.getIndexer().drive(-1);
                // else robot.getIndexer().drive(0);

                // robot.getFrontIntake().update(getGamepad1().getLeftBumper());

                //  if(getGamepad1().getXButton()) robot.getRearIntake().drive(true, false);
                //  else if(getGamepad1().getYButton()) robot.getRearIntake().drive(true, true);
                //  else robot.getRearIntake().drive(false, false);

                //  // Change gear w/ cooldown
                // if (getGamepad1().getRightBumper() && lastGearChange + GEAR_CHANGE_CD < System.currentTimeMillis()) {
                //     robot.getDriveTrain().changeGear();
                //     lastGearChange = System.currentTimeMillis();
                // }

              //  robot.getFrontIntake().printColourVals();

                // if(getGamepad1().getLeftBumper()) robot.getFrontIntake().drive(1);
                // else if(getGamepad1().getRightBumper()) robot.getFrontIntake().drive(-1);
                // else robot.getFrontIntake().drive(0);

                // robot.getVision().printDistanceToGoal();


//TEST CODE ENDS



                // Indexer
                // #drive(enabled = a or b is pressed, reversed = b is pressed)

                // Front intake drive
                // #drive(enabled = left bumper or right bumper is pressed, reversed = left
                // bumper is pressed)

                // Front intake articulatess

                // Rear intake drive
                // #drive(enabled = x or y is pressed, reversed = x is pressed)
                // robot.getRearIntake().drive(getGamepad1().getXButton() || getGamepad1().getYButton(),
                //         getGamepad1().getXButton());

                // // Drivetrain
                // #arcadeDrive(speed = right trigger - left trigger, turn = left joystick x
                //  axis)
                robot.getDriveTrain().arcadeDrive(
                        getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(),
                        getGamepad1().getLeftX());

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
