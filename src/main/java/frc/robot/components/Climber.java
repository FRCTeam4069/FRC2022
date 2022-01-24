// Class for the 2022 FRC 4069 climber component

package frc.robot.components;

import frc.robot.components.RobotComponent;
import frc.robot.Robot;

import static frc.robot.Constants.*;



public class Climber implements RobotComponent{

    private Robot robot;

    // Interface mehtods

    public Climber(Robot robot) {
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

    public void stop() {
    
    }


}


