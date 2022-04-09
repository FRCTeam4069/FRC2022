package frc.robot.auto.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.Command;

public class AutoAlignCommand extends Command {

    @Override
    public void start() {
        robot.getVision().enableLED();
        
    }

    
    double offsetToGoal;
    double noTargetTimer = -1;
    boolean noTargetLast = false;

    @Override
    public void loop() {
        if(robot.getVision().hasTarget()) {
            noTargetLast = false;
            noTargetTimer = -1;
            offsetToGoal = robot.getVision().table.getEntry("tx").getDouble(0.0);

            double error = 0 - offsetToGoal;
            double output = Math.signum(offsetToGoal) * 0.1;

            robot.getDriveTrain().rawPowerSetting(output, -output);
            System.out.println("Aligning, has target");
            System.out.println("Offset: " + offsetToGoal);
        }
        else {
            if(!noTargetLast) {
                noTargetLast = true;
                noTargetTimer = Timer.getFPGATimestamp();
            }
            robot.getDriveTrain().setPower(0, 0);
            offsetToGoal = -100;
            System.out.println("Aligning, no target");
        }
        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(offsetToGoal) < 1.0 || (Timer.getFPGATimestamp() - noTargetTimer > 1 && noTargetTimer != -1);
    }

    @Override
    public void close() {
        robot.getDriveTrain().rawPowerSetting(0, 0);
        // robot.getVision().disableLED();
    }
    
}
