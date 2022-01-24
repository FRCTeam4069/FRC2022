package frc.robot.components;

import frc.robot.Robot;

import static frc.robot.Constants.*;

public class FrontIntake implements RobotComponent {

    private final Robot robot;

    public FrontIntake(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {
        return this;
    }

    @Override
    public void loop() {

    }

    @Override
    public void shutdown() {
        
    }

}