package frc.robot.auto.commands.frontIntake;

import frc.robot.auto.Command;

public class DisableIntakeCommand extends Command {
    @Override
    public void start() {
        robot.getFrontIntake().drive(0);
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
