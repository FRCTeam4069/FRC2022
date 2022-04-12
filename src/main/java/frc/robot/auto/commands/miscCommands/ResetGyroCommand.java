package frc.robot.auto.commands.miscCommands;

import frc.robot.Robot;
import frc.robot.auto.Command;

public class ResetGyroCommand extends Command {
    
    @Override
    public void start() {
        robot.getIMU().resetHeading();
    }

    @Override
    public void loop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void close() {
        // TODO Auto-generated method stub
        
    }
    
}
