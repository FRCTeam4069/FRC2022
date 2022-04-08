package frc.robot.auto.commands.miscCommands;

import frc.robot.auto.Command;

public class ParallelCommand extends Command {

    private Command c1, c2;
    public ParallelCommand(Command c1, Command c2) {
        this.c1 = c1;
        this.c2 = c2;
    }
    @Override
    public void start() {
        c1.setSubsystems(robot);
        c2.setSubsystems(robot);
        c1.start();
        c2.start();   
    }

    @Override
    public void loop() {
        if(!c1.isFinished()) c1.loop();
        if(!c2.isFinished()) c2.loop();
    }

    @Override
    public boolean isFinished() {
        return c1.isFinished() && c2.isFinished();
    }

    @Override
    public void close() {
        c1.close();
        c2.close();
    }
    
}
