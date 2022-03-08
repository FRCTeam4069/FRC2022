package frc.robot.auto.commands.frontIntake;

import frc.robot.auto.Command;

public class EnableIntakeCommand extends Command {

    @Override
    public void start() {
        robot.getFrontIntake().drive(1);
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
