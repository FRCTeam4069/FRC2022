package frc.robot;


import com.revrobotics.AnalogInput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Scheduler.RobotRepeatingTask;
import frc.robot.subsystems.FrontIntake;
import frc.robot.subsystems.DriveTrain.direction;

/** Controls handlingß */
public class Controls {

    // IDs
    private static final int GP_1 = 0, GP_2 = 1;

    private static final double DRIVETRAIN_TRIGGER_DEADBAND = 0.05;
    private static final double DRIVETRAIN_STICK_DEADBAND = 0.1;

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


    boolean manualAdjustment = false;

    int climbingSequencerCount = 0;
    boolean climbing = false;
    boolean firstSequenceFive = true;
    double sequenceFiveTimer = 0;
    boolean fiveRetracted = false;
    boolean fiveFired = false;

    boolean currentStopFive = false;
    boolean currentSpikeLast = false;
    double currentSpikeStartTime = 0;

    boolean firstSequenceEight = true;
    double sequenceEightTimer = 0;
    boolean eightRetracted = false;
    boolean eightFired = false;

    boolean shortFired = true;
    boolean longFired = true;

    int lastDPAD = -1;

    double autoAlignStartTime = 0;
    boolean autoAlignStarted = false;
    boolean autoIdexerOut = false;

    double timeSincePressed;
    boolean FrontIntakeOUT = false;
    boolean FrontIntakeIN = false;

    boolean stopDriveForClimb = false;


    direction WhereRobotGoin;

    /**
     * Requires robot dependancy
     * 
     * @param robot Robot instance
     */
    public Controls(Robot robot) {
        this.robot = robot;

       
        SmartDashboard.putNumber("lowerRPM", 600);
        SmartDashboard.putNumber("upperRPM", 750);
        SmartDashboard.putNumber("SimDist", 0);
        SmartDashboard.putBoolean("Auto Shoot?", false);
        
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
                 * A - Indexer up when loaded
                 * Y - Indexer down when loaded
                 * 
                 */

                if(!climbing) {
                    robot.getVision().enableLED();
                    robot.getFlywheel().atSpeed();
                    robot.getDriveTrain().updateDirection();
                    WhereRobotGoin = robot.getDriveTrain().getDirection();

                    double lowerRPM = SmartDashboard.getNumber("lowerRPM", 750);
                    double upperRPM = SmartDashboard.getNumber("upperRPM", 750);
                    Boolean shoootFas = SmartDashboard.getBoolean("Auto Shoot?", false);
                    
                    SmartDashboard.putNumber("Current Distance", robot.getVision().getDistance());
                    SmartDashboard.putBoolean("AnalogSensor", robot.getIndexer().getSensor());
                    SmartDashboard.putNumber("CurrentDTopSpeed", robot.getFlywheel().getTopVel());
                    SmartDashboard.putNumber("CurrentDBottomSpeed", robot.getFlywheel().getBottomVel());

                    // 8 feet - about 495 , 700

                     // Drive
                    double rightTrigger = rightTriggerLimiter.calculate(getGamepad1().getRightTriggerAxis());
                    double leftTrigger = leftTriggerLimiter.calculate(getGamepad1().getLeftTriggerAxis());
                    double turn;
                   
                    if(!stopDriveForClimb){}
                    if(robot.getDriveTrain().highGear) turn = turnLimiter.calculate((1.0 / (10.0 / 3.0)) * Math.pow(getGamepad1().getLeftX(), 3));
                    else turn = turnLimiter.calculate(0.5 * Math.pow(getGamepad1().getLeftX(), 3));
                    
                   // System.out.println(turn);
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

                    if(getGamepad2().getRightY() < -0.25) manualAdjustment = true;

                    if(getGamepad2().getRightTriggerAxis() > 0.5) robot.getFrontIntake().driveIntakeOnly(-1);
                    else if(getGamepad2().getLeftTriggerAxis() > 0.5) robot.getFrontIntake().driveIntakeOnly(1);
                    else robot.getFrontIntake().driveIntakeOnly(0);

                    if(getGamepad2().getRightY() < -0.25) robot.getFrontIntake().rawArticulate(0.25);
                        else if(getGamepad2().getRightY() > 0.25) robot.getFrontIntake().rawArticulate(-0.25);
                        else robot.getFrontIntake().rawArticulate(0);



                    // Rear Intake
                    robot.getRearIntake().drive(getGamepad2().getLeftBumper() || getGamepad2().getRightBumper(),
                            getGamepad2().getLeftBumper(), robot.getFrontIntake());

                    // Indexer
                    if(getGamepad2().getLeftBumper()) robot.getIndexer().drive(1);
                    else if(getGamepad2().getLeftTriggerAxis() > 0.5) robot.getIndexer().drive(1);
                    if(!robot.getIndexer().getSensor()){
                        if(getGamepad2().getLeftY() > 0.5) robot.getIndexer().drive(1);
                        else if(getGamepad2().getLeftY() < -0.5) { robot.getIndexer().drive(-1);robot.getFrontIntake().driveIntakeOnly(1);}
                        else robot.getIndexer().drive(0);}
                    else if(autoIdexerOut) robot.getIndexer().drive(1);
                    else if(getGamepad2().getAButton()){robot.getIndexer().drive(-1);}
                    else if(getGamepad2().getYButton()){robot.getIndexer().drive(1);}
                    else robot.getIndexer().drive(0);

                    
                    // Shooter
                    boolean startedShootingProcess = false;
                    boolean enableLED = true;

                    int currentDPAD = getGamepad1().getPOV(0);
                    if(currentDPAD != lastDPAD) {
                        lastDPAD = currentDPAD;
                        if(currentDPAD != -1) {
                            if(currentDPAD == 180 || currentDPAD == 135 || currentDPAD == 225) {
                                robot.getFlywheel().constant -= 10;
                                
                            }
                            else if(currentDPAD == 0 || currentDPAD == 315 || currentDPAD == 45) {
                                robot.getFlywheel().constant += 10;
                            }
                        }
                    }

                    if(getGamepad1().getAButton()) {

                        if(!autoAlignStarted) {
                            autoAlignStarted = true;
                            autoAlignStartTime = Timer.getFPGATimestamp();
                        }

                        if(autoAlignStartTime != 0 && autoAlignStartTime + 0.5 > Timer.getFPGATimestamp()) autoIdexerOut = true;
                        else autoIdexerOut = false;

                       // intakeUp = false;
                        shooterIntakeLockout = true;
                        enableLED = true;

                        if(!startedShootingProcess) {
                            startedShootingProcess = true;
                            robot.getDriveTrain().resetTurnError();
                        }

                        if(Math.abs(robot.getDriveTrain().getTurnError()) > 1.75) {
                            robot.getDriveTrain().alignToGoal();
                        }
                        else {
                            robot.getDriveTrain().stop();
                        }
                    }
                    else {
                        autoAlignStarted = false;
                        autoIdexerOut = false;
                        autoAlignStartTime = 0;

                        startedShootingProcess = false;
                        robot.getDriveTrain().alignFirstTime = true;
                        robot.getDriveTrain().endLockout();
                        robot.getFrontIntake().shooterLock = false;
                        shooterIntakeLockout = false;
                    }

                    if(getGamepad1().getXButton()) {
                        enableLED = true;
                        shooterIntakeLockout = true;
                        timeSincePressed = Timer.getFPGATimestamp();
                        
                        if(!robot.getVision().hasTarget()) {
                            System.out.println("no target, assuming close shot");
                            robot.getFlywheel().update(750 , 425);
                        }
                        else robot.getFlywheel().updateDistance(robot.getVision().getDistance(), false, lowerRPM);
                    }
                    else {if(!funkyShot) robot.getFlywheel().updatePercentage(0, 0);}
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
                        robot.getFlywheel().update(upperRPM  , lowerRPM);
                        funkyShot = true;
                    }
                    else funkyShot = false;

                }

                if(getGamepad1().getBackButtonPressed()) {
                    if(shortFired) robot.getClimber().retractShort();
                    else robot.getClimber().fireShort();
                    System.out.println(shortFired);
                    shortFired = !shortFired;
                }
                if(getGamepad1().getRightStickButtonPressed()) {
                    if(longFired) robot.getClimber().retractLong();
                    else robot.getClimber().fireLong();
                    System.out.println(longFired);
                    longFired =  !longFired;
                }

                //Climber stufffffff

                if(getGamepad1().getStartButtonPressed()) climbingSequencerCount++;


                // Arms flip up 90 degrees
                if(climbingSequencerCount == 1) {
                    robot.getClimber().update(90, false);
                    shooterIntakeLockout = true;
                  //  robot.getFrontIntake().dropForShot();
                }

                // Rotate up to grab high bar, stp at current spike or press
                else if(climbingSequencerCount == 2 && robot.getClimber().getCurrent() < 70) {
                    robot.getDriveTrain().stop();
                    robot.getIndexer().drive(0);
                    robot.getFlywheel().updatePercentage(0, 0);
                    robot.getRearIntake().drive(false, false, robot.getFrontIntake());
                    robot.getFrontIntake().rawArticulate(0);
                    robot.getFrontIntake().driveIntakeOnly(0);
                    climbing = true;
                    robot.getClimber().test(-1);
                }

                // No power, suspended between mid and high
                else if(climbingSequencerCount == 3 || (robot.getClimber().getCurrent() >= 70 && climbingSequencerCount == 2)) {
                    climbingSequencerCount = 3;
                    double power = 0;
                    if(getGamepad1().getRightY() > 0.25) power = -1;
                    else if(getGamepad1().getRightY() < -0.25) power = 1;
                    robot.getClimber().test(power);
                }

                // Arms drive back and hooks release, manual control to drive it back if needed, high hang
                else if(climbingSequencerCount == 4) {
                    if(firstSequenceFive) {
                        firstSequenceFive = false;
                        sequenceFiveTimer = Timer.getFPGATimestamp();
                    }
                    if(Timer.getFPGATimestamp() < sequenceFiveTimer + 0.75) {
                        robot.getClimber().test(1);
                    }

                    //Move to later
                    // else if(Timer.getFPGATimestamp() >= sequenceFiveTimer + 0.75) {
                        
                    //     robot.getClimber().test(-1);
                    // }
                    if(!fiveRetracted && Timer.getFPGATimestamp() > sequenceFiveTimer + 0.6) {
                        robot.getClimber().fireLong();
                        fiveRetracted = true;
                    }

                    if(Timer.getFPGATimestamp() >= sequenceFiveTimer + 0.75) {
                        double output = 0;
                        if(getGamepad1().getRightY() < -0.25) output = 1;
                        robot.getClimber().test(output);
                    }

                    //Move to later
                    // if(Timer.getFPGATimestamp() > sequenceFiveTimer + 1.5 && !fiveFired) {
                    //     fiveFired = true;
                    //     robot.getClimber().retractLong();
                    // }
                }

                // Hooks fire back closed, rotate up to grab traversal bar, stop on press or current
                if(climbingSequencerCount == 5 && !currentStopFive) {
                    if(!fiveFired) {
                        fiveFired = true;
                        robot.getClimber().retractLong();
                    }

                    if(robot.getClimber().getCurrent() > 70 && !currentSpikeLast) {
                        currentSpikeStartTime = Timer.getFPGATimestamp();
                        currentSpikeLast = true;
                    }
                    else if(robot.getClimber().getCurrent() > 70 && currentSpikeLast) {
                        if(Timer.getFPGATimestamp() > 0.5 + currentSpikeStartTime) currentStopFive = true;
                    }
                    else {
                        currentSpikeStartTime = 0;
                        currentSpikeLast = false;
                    }

                    robot.getClimber().test(-1);
                }
                //Suspended between high and traversal
                else if(climbingSequencerCount == 6 || (currentStopFive && climbingSequencerCount == 5)) {
                    climbingSequencerCount = 6;
                    double power = 0;
                    if(getGamepad1().getRightY() > 0.25) power = -1;
                    else if(getGamepad1().getRightY() < -0.25) power = 1;
                    robot.getClimber().test(power);
                }

                // else if(climbingSequencerCount == 7) {
                //     double output = 0;
                //     if(getGamepad2().getRightY() > 0.25) output = 1;
                //     else if(getGamepad2().getRightY() < -0.25) output = -1;
                //     robot.getClimber().test(output);
                // }

                // Drives back and disengages, manual control ensues should further backdriving be required
                else if(climbingSequencerCount > 6) {

                    if(firstSequenceEight) {
                        firstSequenceEight = false;
                        sequenceEightTimer = Timer.getFPGATimestamp();
                    }

                    if(Timer.getFPGATimestamp() < sequenceEightTimer + 1) {
                        robot.getClimber().test(1);
                    }
                    if(!eightRetracted && Timer.getFPGATimestamp() > sequenceEightTimer + 0.6) {
                        robot.getClimber().retractShort();
                        eightRetracted = true;
                    }
    
                    if(Timer.getFPGATimestamp() >= sequenceEightTimer + 1) {
                        double output = 0;
                        if(getGamepad1().getRightY() < -0.25) output = 1;
                        else if(getGamepad1().getRightY() > 0.25) output = -1;
                        robot.getClimber().test(output);

                        if(getGamepad1().getBackButtonPressed()) robot.getClimber().retractShort();
                    }
                }
                

                if(getGamepad1().getLeftStickButton()) {
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

                if(getGamepad1().getYButton()) {
                    robot.getClimber().test(0);
                }

                SmartDashboard.putString("Work in controls?", "You betcha");

             //   System.out.println("Current Step: " + climbingSequencerCount);

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

               //  robot.getFrontIntake().rawArticulate(getGamepad1().getRightY());

                    // Indexer
                    // if(getGamepad1().getLeftBumper()) robot.getIndexer().drive(1);
                    // else if(getGamepad1().getLeftTriggerAxis() > 0.5) robot.getIndexer().drive(1);
                    // else if(getGamepad1().getLeftY() > 0.5) robot.getIndexer().drive(1);
                    // else if(getGamepad1().getLeftY() < -0.5) robot.getIndexer().drive(-1);
                    // else robot.getIndexer().drive(0);

                    ///////INTAKE TESTING CODE IN THE NEXT COMMENT BLOCK

                if(getGamepad1().getLeftTriggerAxis() > 0.25) robot.getFrontIntake().driveIntakeOnly(1);
                else if(getGamepad1().getRightTriggerAxis() > 0.25) robot.getFrontIntake().driveIntakeOnly(-1);
                else robot.getFrontIntake().driveIntakeOnly(0);

                if(getGamepad1().getRightY() < -0.25) robot.getFrontIntake().rawArticulate(0.3);
                else if(getGamepad1().getRightY() > 0.25) robot.getFrontIntake().rawArticulate(-0.3);
                else robot.getFrontIntake().rawArticulate(0);

                System.out.println("Velocity of intake (tick/sec): " + robot.getFrontIntake().getVel());
                // if(getGamepad1().getBackButton()) robot.getFrontIntake().dropForShot();
                // else if(getGamepad1().getStartButton()) robot.getFrontIntake().raise();
                // else robot.getFrontIntake().rawArticulate(0);
                // robot.getFrontIntake().printColourVals();

                //  if(getGamepad1().getXButton()) robot.getFlywheel().update(1300, 1080);
                //  else if(getGamepad1().getBButton()) robot.getFlywheel().update(1300, 1090);
                //  else if(getGamepad1().getYButton()) robot.getFlywheel().update(1300, 1085);
                //  else robot.getFlywheel().updatePercentage(0, 0);

               // if(getGamepad1().getAButton()) robot.getDriveTrain().resetPos();
                // robot.getDriveTrain().updatePos();
                // System.out.println("X: " + robot.getDriveTrain().getPose().getX());
                // System.out.println("Y: " + robot.getDriveTrain().getPose().getY());
                // System.out.println("Theta: " + robot.getDriveTrain().getPose().getRotation().getDegrees());
        //         if(getGamepad1().getRightY() < -0.25) output = 1;
        //         else if(getGamepad1().getRightY() > 0.25) output = -1; 
        //         //robot.getClimber().test(output);

        //   //     robot.getClimber().update(180, false);
        //         robot.getDriveTrain().stop();
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
                // robot.getDriveTrain().arcadeDrive(
                //         getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(),
                //         getGamepad1().getLeftX());

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
