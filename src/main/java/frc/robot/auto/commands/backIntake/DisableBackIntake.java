package frc.robot.auto.commands.backIntake;

import frc.robot.auto.Command;

public class DisableBackIntake extends Command {

    @Override
    public void start() {
        robot.getRearIntake().drive(false, false, robot.getFrontIntake());
    }

    @Override
    public void loop() { }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void close() {}
    
}