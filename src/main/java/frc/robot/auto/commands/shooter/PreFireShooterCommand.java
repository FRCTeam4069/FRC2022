package frc.robot.auto.commands.shooter;

import frc.robot.auto.Command;

public class PreFireShooterCommand extends Command {

    double topVelRPM;
    double bottomVelRPM;

    public PreFireShooterCommand(double topVelRPM, double bottomVelRPM) {
        this.topVelRPM = topVelRPM;
        this.bottomVelRPM = bottomVelRPM;
    }

    @Override
    public void start() {
        robot.getFlywheel().update(topVelRPM, bottomVelRPM);
        
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void close() {
        // TODO Auto-generated method stub
        
    }
    
}
