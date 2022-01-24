package frc.robot.components;

import frc.robot.Robot;

/**
 * Interface for components
 */
public interface RobotComponent {
    
    /**
     * Called upon robot initialization.
     * 
     * @return Instance of itself
     */
    public RobotComponent init();

    /**
     * Loop method
     */
    public void loop();

    /**
     * Called for closing motors, etc at end
     */
    public void shutdown();

}
