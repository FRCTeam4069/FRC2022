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

    /** Called before first command loop */
    public abstract void start();

    /** Called every loop of auto */
    public abstract void loop();

    /** Return true when command should no longer be run */
    public abstract boolean isFinished();

    /** Cleanup to be run after isFinished returns true */
    public abstract void close();
}

