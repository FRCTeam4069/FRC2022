package frc.robot.auto.commands.backIntake;

import frc.robot.auto.Command;

public class EnableBakIntake extends Command {

    @Override
    public void start() {
        robot.getRearIntake().drive(true, true);
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
