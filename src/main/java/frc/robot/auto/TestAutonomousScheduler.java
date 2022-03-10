package frc.robot.auto;

import frc.robot.auto.commands.frontIntake.DisableIntakeCommand;
import frc.robot.auto.commands.frontIntake.EnableIntakeCommand;
import frc.robot.auto.commands.miscCommands.WaitCommand;
import frc.robot.auto.commands.shooter.ShootCommand;
import frc.robot.Robot;
import frc.robot.auto.commands.vision.EnableLimelightLEDCommand;

public class TestAutonomousScheduler implements AutoRoutine {

	AutoScheduler aScheduler;

    Robot robot;

    public TestAutonomousScheduler(Robot robot) {
        this.robot = robot;
    }

    @Override
    public String name() {
        return "Test Autonomous Scheduler";
    }

    @Override
    public void init() {
        aScheduler = new AutoScheduler(robot);
        aScheduler.addCommand(new EnableLimelightLEDCommand());
        aScheduler.addCommand(new WaitCommand(0.5));
        aScheduler.addCommand(new DisableIntakeCommand());
        aScheduler.addCommand(new WaitCommand(0.25));
        aScheduler.addCommand(new EnableIntakeCommand());
        aScheduler.addCommand(new WaitCommand(0.5));
        aScheduler.addCommand(new DisableIntakeCommand());
        aScheduler.addCommand(new WaitCommand(0.25));
        aScheduler.addCommand(new ShootCommand(800, 800, 5));

        
    }

    @Override
    public void loop() {
       aScheduler.run(); 
    }
    
}
