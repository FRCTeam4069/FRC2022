package frc.robot.auto;

import frc.robot.Robot;

/**
 * Test auto routine class
 * Acts as an example of how to create an auto routine using the AutoRoutine interface
 * Also good for testing out different parts
 */
public class TestAuto implements AutoRoutine {

    private final Robot robot;

    // Dependency injection (allows us to use methods from robot class)
    public TestAuto(Robot robot) {
        this.robot = robot;
    }

    // Auto mode name
    @Override
    public String name() {
        return "Test Auto";
    }

    // Initialization of routine
    @Override
    public void init() {
        // TODO: Auto-generated method stub
    }

    // Loops every 20ms
    @Override
    public void loop() {
        // TODO Auto-generated method stub
    }
    
}
