// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Team 4069
// Lo-Ellen Robotics
// Greater Sudbury, ON
// #girlsinSTEM

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.AutoRoutine;
import frc.robot.auto.TestAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.FrontIntake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.RearIntake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot 
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	// SET GYRO ID!
	private static final int GYRO_ID = 3;

	// Robot mechanism components
	private DriveTrain driveTrain;
	private Climber climber;
	private Flywheel shooter;
	private RearIntake rearIntake;
	private FrontIntake frontIntake;
	private Indexer indexer;

	// Robot misc. hardware
	private PowerDistribution pdp;
	private PigeonIMU gyro;

	// Robot utils
	private Controls controls;
	private Pneumatics pneumatics;
	private Scheduler scheduler;

	// Auto routine
	private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();
	private final TestAuto testAuto = new TestAuto(this);
	private AutoRoutine autoRoutine = null;

	// Active mode
	private RobotMode mode = RobotMode.DISABLED;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Scheduler init
		scheduler = new Scheduler(this);

		// Send auto selector
		autoChooser.setDefaultOption(testAuto.name(), testAuto);
		SmartDashboard.putData("Select Autonoumous Routine", autoChooser);

		// Subsystem init
		frontIntake = new FrontIntake(this);
		rearIntake = new RearIntake();
		driveTrain = new DriveTrain();
		shooter = new Flywheel(false);
		indexer = new Indexer();

		// Util init
		controls = new Controls(this);
		pneumatics = new Pneumatics(this);
		gyro = new PigeonIMU(GYRO_ID);

		pdp = new PowerDistribution(2, ModuleType.kRev);
		LiveWindow.disableAllTelemetry();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		scheduler.runRepeatingTasks();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the 
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the 
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		mode = RobotMode.AUTO;

		// Gets selected routine
		autoRoutine = autoChooser.getSelected();
		System.out.println("Selected Auto Mode: " + autoRoutine.name());

		// Initializes that routine
		autoRoutine.init();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		// Calls the loop of selected auto
		autoRoutine.loop();
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		mode = RobotMode.TELEOP;
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {

	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		mode = RobotMode.DISABLED;
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {

	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {
		mode = RobotMode.TEST;
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		// driveTrain.tankDrive(getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(), getGamepad1().getLeftX());
		
	}

	/*
	Non-Static getter methods 
	Use dependancy injection to access these in classes
	*/

	/** Gets the drivetrain subsystem
	 */
	public DriveTrain getDriveTrain() {
		return driveTrain;
	}

	/** Gets the climber subsystem */
	public Climber getClimber() {
		return climber;
	}
	
	/** Gets the flywheel (shooter) subsystem */
	public Flywheel getFlywheel() {
		return shooter;
	}

	/** Gets the rear intake subsystem */
	public RearIntake getRearIntake() {
		return rearIntake;
	}

	/** Gets the front intake subsystem */
	public FrontIntake getFrontIntake() {
		return frontIntake;
	}

	/** Gets the active indexer subsystem */
	public Indexer getIndexer() {
		return indexer;
	}

	/** Gets the active control mapping class */
	public Controls getControls() {
		return controls;
	}

	/** Gets the active pneumatics handling class */
	public Pneumatics getPneumatics() {
		return pneumatics;
	}

	/** Gets the current mode of the robot */
	public RobotMode getMode() {
		return mode;
	}

	/** Gets the voltage as read by the PDP */
	public double getBatteryVoltage() {
		return pdp.getVoltage();
	}

	/** Gets the PigeonIMU gyroscope */
	public PigeonIMU getGyroscope() {
		return gyro;
	}

	/** Gets the scheduler */
	public Scheduler getScheduler() {
		return scheduler;
	}

	/** Modes the robot can be put in */
	public enum RobotMode {
		DISABLED,
		TEST,
		AUTO,
		TELEOP;
	}
}
