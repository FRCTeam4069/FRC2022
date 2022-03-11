package frc.robot.auto.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.Command;

public class ShootCommand extends Command {

    private double topWheelSpeed;
    private double bottomWheelSpeed;
    private double duration;

    private double startTime;

    /**
     * Setup shoot command
     * 
     * @param topWheelSpeed in RPM - should probably be 1350
     * @param bottomWheelSpeed in RPM
     * @param duration in seconds
     */
    public ShootCommand(double topWheelSpeed, double bottomWheelSpeed, double duration) {
        this.topWheelSpeed = topWheelSpeed;
        this.bottomWheelSpeed = bottomWheelSpeed;
        this.duration = duration;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void loop() {
        robot.getFlywheel().update(topWheelSpeed, bottomWheelSpeed);
        robot.getIndexer().drive(1);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > duration;
    }

    @Override
    public void close() {
        robot.getFlywheel().updatePercentage(0, 0);
        robot.getIndexer().drive(0);
    }
    
}
