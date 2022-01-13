package frc.robot.components;

import frc.robot.Robot;

/**
 * Interface for components
 */
public interface RobotComponent {
    
    /**
     * Called upon robot initialization.
     * 
     * @param robot Robot instance for dependancy injection
     * 
     * @return Instance of itself
     */
    public RobotComponent init(Robot robot);

    /**
     * Called for closing motors, etc at end
     */
    public void shutdown();

}
