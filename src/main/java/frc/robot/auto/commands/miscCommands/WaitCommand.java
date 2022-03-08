package frc.robot.auto.commands.miscCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.Command;

public class WaitCommand extends Command {

    private double duration;
    private double startingTime;

    /**
     * Set up wait command
     * @param duration in seconds
     */
    public WaitCommand(double duration) {
        this.duration = duration;
    }

    @Override
    public void start() {
        startingTime = Timer.getFPGATimestamp();
    }

    @Override
    public void loop() {
        System.out.println("Waiting...");
        
    }

    @Override
    public boolean isFinished() {
        return (startingTime - Timer.getFPGATimestamp()) > duration;
    }

    @Override
    public void close() {
    }
    
}
