// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.AutoRoutine;
import frc.robot.auto.TestAuto;
import frc.robot.components.Climber;
import frc.robot.components.DriveTrain;
import frc.robot.components.FrontIntake;
import frc.robot.components.shooter.Flywheel;
import frc.robot.components.RearIntake;

import static frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	// Components
	private DriveTrain driveTrain;
	private Climber climber;
	private Flywheel flywheel;
	private RearIntake rearIntake;
	private FrontIntake frontIntake;

	// Controllers
	private XboxController controller1;
	private XboxController controller2;

	// Auto routine
	private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();
	private final TestAuto testAuto = new TestAuto(this);
	private AutoRoutine autoRoutine = null;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Send auto selector
		autoChooser.setDefaultOption(testAuto.name(), testAuto);
		SmartDashboard.putData("Select Autonoumous Routine", autoChooser);

		// Controller init
		controller1 = new XboxController(GP1_USB);
		controller2 = new XboxController(GP2_USB);

		// Component init
		frontIntake = (FrontIntake) new FrontIntake().init();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and
	 * test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different
	 * autonomous modes using the dashboard. The sendable chooser code works with
	 * the Java
	 * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
	 * chooser code and
	 * uncomment the getString line to get the auto name from the text box below the
	 * Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure
	 * below with additional strings. If using the SendableChooser make sure to add
	 * them to the
	 * chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
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
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {

	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {

	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {

	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		
	}

	/*
	Non-Static getter methods 
	**Use dependancy injection to access these in classes**
	*/

	/**
	 * Gets access to the active DriveTrain component
	 * @return Active DriveTrain
	 */
	public DriveTrain getDriveTrain() {
		return driveTrain;
	}

	/**
	 * Gets access to the active Climber component
	 * @return Active Climber
	 */
	public Climber getClimber() {
		return climber;
	}
	
	/**
	 * Gets access to the active Flywheel component
	 * @return Active Flywheel
	 */
	public Flywheel getFlywheel() {
		return flywheel;
	}

	/**
	 * Gets access to the active Front Intake component
	 * @return Active Rear Intake
	 */
	public RearIntake getRearIntake() {
		return rearIntake;
	}

	/**
	 * Gets access to the active Front Intake component
	 * @return Active Front Intake
	 */
	public FrontIntake getFrontIntake() {
		return frontIntake;
	}

	/**
	 * Gets the connected gamepad (1)
	 * @return 1st Gamepad
	 */
	public XboxController getGamepad1() {
		return controller1;
	}

	/**
	 * Gets the connected gamepad (2)
	 * @return 2st Gamepad
	 */
	public XboxController getGamepad2() {
		return controller2;
	}
}
