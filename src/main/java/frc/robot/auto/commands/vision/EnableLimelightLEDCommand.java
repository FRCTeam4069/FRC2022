package frc.robot.auto.commands.vision;

import frc.robot.auto.Command;

public class EnableLimelightLEDCommand extends Command {

    @Override
    public void start() {
        robot.getVision().enableLED();
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void close() {}
    
}
