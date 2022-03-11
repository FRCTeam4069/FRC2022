package frc.robot.auto.commands.indexer;

import frc.robot.auto.Command;

public class DisableIndexer extends Command {

    @Override
    public void start() {
        robot.getIndexer().drive(0);
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