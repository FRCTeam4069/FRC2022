package frc.robot.auto.commands.indexer;

import frc.robot.auto.Command;

public class EnableIndexer extends Command {

    private double power;
    /** -1 is fully in, 1 is fully out */
    public EnableIndexer(double power) {
        this.power = power;
    }

    @Override
    public void start() {
        robot.getIndexer().drive(power);
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
