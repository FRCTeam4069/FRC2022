package frc.robot.components;

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

}
