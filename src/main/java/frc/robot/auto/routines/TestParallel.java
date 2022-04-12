package frc.robot.auto.routines;

import frc.robot.Robot;
import frc.robot.auto.AutoScheduler;
import frc.robot.auto.commands.backIntake.DisableBackIntake;
import frc.robot.auto.commands.backIntake.EnableBakIntake;
import frc.robot.auto.commands.indexer.DisableIndexer;
import frc.robot.auto.commands.indexer.EnableIndexer;
import frc.robot.auto.commands.miscCommands.ParallelCommand;
import frc.robot.auto.commands.miscCommands.WaitCommand;

public class TestParallel implements AutoRoutine {

    Robot robot;
    AutoScheduler scheduler;
    public TestParallel(Robot robot) {
        this.robot = robot;
        scheduler = new AutoScheduler(robot);
    }

    @Override
    public String name() {
        // TODO Auto-generated method stub
        return "Parallel";
    }

    @Override
    public void init() {
        scheduler.addCommand(new ParallelCommand(new EnableIndexer(-1), new EnableBakIntake()));
        scheduler.addCommand(new WaitCommand(0.5));
        scheduler.addCommand(new ParallelCommand(new DisableIndexer(), new DisableBackIntake()));
    }

    @Override
    public void loop() {
       scheduler.run();
        
    }
    
}
