package frc.robot.auto;

import frc.robot.Robot;

public abstract class Command {

    //Subsystems to be accessed by commands
    protected Robot robot;

    //MUST be called by a command scheduler and runner
    public void setSubsystems(Robot robot) {
        this.robot = robot;
    }

    //Methods that need to be implemented by each command
    public abstract void start();
    public abstract void loop();
    public abstract boolean isFinished();
    public abstract void close();
}

