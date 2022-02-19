// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Team 4069
// Lo-Ellen Robotics
// Greater Sudbury, ON

// #girlsinSTEM
// #transgirlsinSTEM

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.AutoRoutine;
import frc.robot.auto.TestAuto;
import frc.robot.components.Climber;
import frc.robot.components.Controls;
import frc.robot.components.DriveTrain;
import frc.robot.components.FrontIntake;
import frc.robot.components.Pneumatics;
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

	// Robot mechanism components
	private DriveTrain driveTrain;
	private Climber climber;
	private Flywheel flywheel;
	private RearIntake rearIntake;
	private FrontIntake frontIntake;

	private PowerDistribution pdp;

	// Robot util. components
	private Controls controls;
	private Pneumatics pneumatics;

	// Auto routine
	private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();
	private final TestAuto testAuto = new TestAuto(this);
	private AutoRoutine autoRoutine = null;

	private RobotMode mode = RobotMode.DISABLED;

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

		// Component init
		frontIntake = (FrontIntake) new FrontIntake().init();
		rearIntake = (RearIntake) new RearIntake().init();
		driveTrain = (DriveTrain) new DriveTrain().init();
		controls = (Controls) new Controls(this).init();

		pdp = new PowerDistribution(0, ModuleType.kRev);
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
		driveTrain.tankDrive(getGamepad1().getRightTriggerAxis() - getGamepad1().getLeftTriggerAxis(), getGamepad1().getLeftX());
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
	 * Gets access to the active Controller input component
	 * @return Active Controls
	 */
	public Controls getControls() {
		return controls;
	}

	/**
	 * Gets access to the active Pneumatics component
	 * @return Active Pneumatics
	 */
	public Pneumatics getPneumatics() {
		return pneumatics;
	}

	/**
	 * Gets the connected gamepad (1)
	 * @return 1st Gamepad
	 */
	public XboxController getGamepad1() {
		return controls.getGamepad1();
	}

	/**
	 * Gets the connected gamepad (2)
	 * @return 2st Gamepad
	 */
	public XboxController getGamepad2() {
		return controls.getGamepad2();
	}

	/**
	 * Gets the current mode of the robot;
	 * @return Robot's current mode
	 */
	public RobotMode getMode() {
		return mode;
	}

	/**
	 * Get the voltage as read by the PDP
	 * @return Battery Voltage
	 */
	public double getBatteryVoltage() {
		return pdp.getVoltage();
	}

	/** 
	 * Various modes
	 */
	public enum RobotMode {
		DISABLED,
		TEST,
		AUTO,
		TELEOP;
	}
}
