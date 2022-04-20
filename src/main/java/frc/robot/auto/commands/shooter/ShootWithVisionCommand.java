package frc.robot.auto.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.Command;

public class ShootWithVisionCommand extends Command {

    private double defaultTopWheelSpeed;
    private double defaultBottomWheelSpeed;
    private double duration;

    private double startTime;

    /**
     * Setup shoot command
     * 
     * @param topWheelSpeed in RPM - should probably be 1300
     * @param bottomWheelSpeed in RPM
     * @param duration in seconds
     */
    public ShootWithVisionCommand(double defaultTopWheelSpeed, double defaultBottomWheelSpeed, double duration) {
        this.defaultTopWheelSpeed = defaultTopWheelSpeed;
        this.defaultBottomWheelSpeed = defaultBottomWheelSpeed;
        this.duration = duration;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void loop() {
        double distance;
        if(robot.getVision().hasTarget()) {
            distance = robot.getVision().getDistance();
            robot.getFlywheel().updateDistance(distance);
        } 
        else {
            robot.getFlywheel().update(defaultTopWheelSpeed, defaultBottomWheelSpeed);
            if(robot.getFlywheel().getTopVel() > defaultTopWheelSpeed) robot.getIndexer().drive(-1);
            return;
        } 

        
        if(robot.getFlywheel().getTopVel() > 1275  || Timer.getFPGATimestamp() > startTime + 2) robot.getIndexer().drive(-1);
        
        
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