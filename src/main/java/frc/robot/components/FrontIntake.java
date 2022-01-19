package frc.robot.components;

import frc.robot.Robot;

import static frc.robot.Constants.*;

public class FrontIntake implements RobotComponent {
    private Robot robot;

    @Override
    public RobotComponent init(Robot robot) {
        this.robot = robot;
        return this;
    }

    @Override
    public void shutdown() {
        
    }

}