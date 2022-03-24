package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Scheduler.RobotRepeatingTask;

/** Controls handlingß */
public class Controls {

    // IDs
    private static final int GP_1 = 0, GP_2 = 1;

    private static final double DRIVETRAIN_TRIGGER_DEADBAND = 0.05;
    private static final double DRIVETRAIN_STICK_DEADBAND = 0.1;
    private static final double INDEXER_DEADBAND = 0.2;

    // Cooldowns (ms)
    private static final int GEAR_CHANGE_CD = 1000; // 1s

    private final Robot robot;

    // Controllers
    private XboxController controller1, controller2;

    // Call timestamps
    private long lastGearChange = 0;

    SlewRateLimiter leftTriggerLimiter = new SlewRateLimiter(0.9);
    SlewRateLimiter rightTriggerLimiter = new SlewRateLimiter(0.9);
    SlewRateLimiter turnLimiter = new SlewRateLimiter(1.8);

    private boolean downforce = false;
    private boolean horizontal = true;

    int climbingSequencerCount = 0;
    boolean climbing = false;
    boolean firstSequenceFive = true;
    double sequenceFiveTimer = 0;
    boolean fiveRetracted = false;
    boolean fiveFired = false;

    boolean firstSequenceEight = true;
    double sequenceEightTimer = 0;
    boolean eightRetracted = false;
    boolean eightFired = false;

    boolean shortFired = true;
    boolean longFired = true;

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
    boolean funkyShot = false;
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

                if(!climbing) {

                    // Drive
                    double rightTrigger = rightTriggerLimiter.calculate(getGamepad1().getRightTriggerAxis());
                    double leftTrigger = leftTriggerLimiter.calculate(getGamepad1().getLeftTriggerAxis());
                    double turn;
                    if(robot.getDriveTrain().highGear) turn = turnLimiter.calculate((1.0 / (10.0 / 3.0)) * Math.pow(getGamepad1().getLeftX(), 3));
                    else turn = turnLimiter.calculate(0.5 * Math.pow(getGamepad1().getLeftX(), 3));

                    System.out.println(turn);
                    if(getGamepad1().getBackButton()) turn = -0.225;
                    else if(getGamepad1().getStartButton()) turn = 0.225;

                    if(climbingSequencerCount == 1 && getGamepad1().getLeftTriggerAxis() > 0.15) leftTrigger = 0.23;
                    else if(climbingSequencerCount == 1) leftTrigger = 0;
                    robot.getDriveTrain().arcadeDrive(
                            MathUtil.applyDeadband(rightTrigger, DRIVETRAIN_TRIGGER_DEADBAND)
                                    - MathUtil.applyDeadband(leftTrigger,
                                            DRIVETRAIN_TRIGGER_DEADBAND),
                            MathUtil.applyDeadband(turn, DRIVETRAIN_STICK_DEADBAND));

                    // Change gear w/ cooldown
                    if (getGamepad1().getRightBumperPressed()
                            && lastGearChange + GEAR_CHANGE_CD < System.currentTimeMillis()) {
                        robot.getDriveTrain().changeGear();
                        lastGearChange = System.currentTimeMillis();
                    }

                    // Front Intake
                    if(getGamepad2().getRightTriggerAxis() > 0.5) robot.getFrontIntake().driveIntakeOnly(-1);
                    else if(getGamepad2().getLeftTriggerAxis() > 0.5) robot.getFrontIntake().driveIntakeOnly(1);
                    else robot.getFrontIntake().driveIntakeOnly(0);

                    if(getGamepad2().getXButtonPressed() && !shooterIntakeLockout) intakeUp = !intakeUp;

                    if(!intakeUp && getGamepad2().getRightY() > 0.25) {
                        downforce = true;
                    }
                    else downforce = false;

                    if(!intakeUp && downforce) {
                        robot.getFrontIntake().driveIntakeOnly(1);
                        robot.getFrontIntake().rawArticulate(-0.2);
                    }
                    else if(intakeUp && !downforce) robot.getFrontIntake().raise();
                    else if(!intakeUp && !downforce) robot.getFrontIntake().dropForShot();

                    
                    

                    // Rear Intake
                    robot.getRearIntake().drive(getGamepad2().getLeftBumper() || getGamepad2().getRightBumper(),
                            getGamepad2().getLeftBumper());

                    // Indexer
                    if(getGamepad2().getLeftBumper()) robot.getIndexer().drive(1);
                    else if(getGamepad2().getLeftY() > 0.5) robot.getIndexer().drive(1);
                    else if(getGamepad2().getLeftY() < -0.5) robot.getIndexer().drive(-1);
                    else robot.getIndexer().drive(0);

                    boolean startedShootingProcess = false;
                    boolean enableLED = false;
                    // Shooter
                    if(getGamepad1().getAButton()) {

                        intakeUp = false;
                        shooterIntakeLockout = true;
                        enableLED = true;

                        if(!startedShootingProcess) {
                            startedShootingProcess = true;
                            robot.getDriveTrain().resetTurnError();
                        //    robot.getDriveTrain().setGear(false);
                        }

                        if(Math.abs(robot.getDriveTrain().getTurnError()) > 3.0) {
                            robot.getDriveTrain().alignToGoal();
                        }
                        else {
                            robot.getDriveTrain().stop();
                        }

                    
                    }
                    else {
                        startedShootingProcess = false;
                        if(!funkyShot) robot.getFlywheel().update(0, 0);
                        robot.getDriveTrain().alignFirstTime = true;
                        robot.getDriveTrain().endLockout();
                        robot.getFrontIntake().shooterLock = false;
                        shooterIntakeLockout = false;
                    }

                    if(getGamepad1().getXButton()) {
                        enableLED = true;
                        shooterIntakeLockout = true;
                        intakeUp = false;
                        if(!robot.getVision().hasTarget()) {
                            System.out.println("no target, assuming close shot");
                            robot.getFlywheel().update(1300, 410);
                        }
                        else robot.getFlywheel().updateDistance(robot.getVision().getDistance());
                    }
                    if(enableLED) { 
                        robot.getVision().enableLED();
                        
                    }
                    else robot.getVision().disableLED();


                    //chip shot
                    if(getGamepad1().getLeftBumper()) {
                        robot.getFlywheel().update(400, 400);
                        funkyShot = true;
                    }
                    else if(getGamepad1().getYButton()) {
                        robot.getFlywheel().update(1300, 800);
                        funkyShot = true;
                    }
                    else funkyShot = false;

                }

                if(getGamepad2().getBackButtonPressed()) {
                    if(shortFired) robot.getClimber().retractShort();
                    else robot.getClimber().fireShort();
                    System.out.println(shortFired);
                    shortFired = !shortFired;
                }
                if(getGamepad2().getRightStickButtonPressed()) {
                    if(longFired) robot.getClimber().retractLong();
                    else robot.getClimber().fireLong();
                    System.out.println(longFired);
                    longFired =  !longFired;
                }

                //Climber stufffffff

                if(getGamepad2().getStartButtonPressed()) climbingSequencerCount++;

                if(climbingSequencerCount == 1) {
                    robot.getClimber().update(90, false);
                    shooterIntakeLockout = true;
                    robot.getFrontIntake().dropForShot();
                }

                else if(climbingSequencerCount == 2 && robot.getClimber().getCurrent() < 70) {
                    robot.getDriveTrain().stop();
                    robot.getIndexer().drive(0);
                    robot.getFlywheel().updatePercentage(0, 0);
                    robot.getRearIntake().drive(false, false);
                    robot.getFrontIntake().rawArticulate(0);
                    robot.getFrontIntake().driveIntakeOnly(0);
                    climbing = true;
                    robot.getClimber().test(-1);
                }

                else if(climbingSequencerCount == 3 || (robot.getClimber().getCurrent() >= 50 && climbingSequencerCount == 2)) {
                    climbingSequencerCount = 3;
                    robot.getClimber().test(0);
                }

                // else if(climbingSequencerCount == 4) {
                //     double output = 0;
                //     if(getGamepad2().getRightY() > 0.25) output = 1;
                //     else if(getGamepad2().getRightY() < -0.25) output = -1;
                //     robot.getClimber().test(output);
                // } 

                else if(climbingSequencerCount == 4 && robot.getClimber().getCurrent() < 70) {
                    if(!fiveRetracted) {
                        robot.getClimber().fireLong();
                        fiveRetracted = true;
                    }
                    if(firstSequenceFive) {
                        firstSequenceFive = false;
                        sequenceFiveTimer = Timer.getFPGATimestamp();
                    }

                    if(Timer.getFPGATimestamp() > sequenceFiveTimer + 0.25) robot.getClimber().test(-1);

                    if(Timer.getFPGATimestamp() > sequenceFiveTimer + 1 && !fiveFired) {
                        robot.getClimber().retractLong();
                        fiveFired = true;
                    }
                }

                else if(climbingSequencerCount == 5 || (robot.getClimber().getCurrent() >= 70 && climbingSequencerCount == 5)) {
                    climbingSequencerCount = 6;
                    robot.getClimber().test(0);
                }

                // else if(climbingSequencerCount == 7) {
                //     double output = 0;
                //     if(getGamepad2().getRightY() > 0.25) output = 1;
                //     else if(getGamepad2().getRightY() < -0.25) output = -1;
                //     robot.getClimber().test(output);
                // }

                else if(climbingSequencerCount > 5) {
                    if(!eightRetracted) {
                        robot.getClimber().retractShort();
                        eightRetracted = true;
                    }
                    double output = 0;
                    if(getGamepad2().getRightY() < -0.25) output = 1;
                    robot.getClimber().test(output);
                }
                

                if(getGamepad2().getLeftStickButton()) {
                    climbingSequencerCount = 0;
                    climbing = false;
                    firstSequenceFive = true;
                    sequenceFiveTimer = 0;
                    fiveRetracted = false;
                    fiveFired = false;
                    firstSequenceEight = true;
                    sequenceEightTimer = 0;
                    eightRetracted = false;
                    eightFired = false;
                }

                if(getGamepad2().getYButton()) {
                    robot.getClimber().test(0);
                }

                System.out.println("Current Step: " + climbingSequencerCount);

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
                double output = 0;
                if(getGamepad1().getRightY() < -0.25) output = 1;
                else if(getGamepad1().getRightY() > 0.25) output = -1; 
                robot.getClimber().test(output);

          //     robot.getClimber().update(180, false);
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
