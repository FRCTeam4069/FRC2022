package frc.robot;

/**
 * Constants that will be used across the robot code will be defined here.
 * 
 * <p>
 * For each class that requires the use of constants, include Constants in your import tag using the follwing example:
 * <p>
 * {@code import static frc.robot.Constants.*;}
 * <p>
 * Methods 
 */
public final class Constants {

    /*
    * Constants are declared as such: 
    * public static final String EXAMPLE_NAME = "Example Value";
    * 
    * These can be referenced across the project by simply typing the name of the constant.
    * Given that these variables will be accessible by any class that statically imports this class,
    * please make sure that the name you assign is specific and will not cause conflict.
    */

    /*
    * Gamepad
    */
    
    public static final int GP1_USB_NUMBER = 0;
    public static final int GP2_USB_NUMBER = 1;
    
    /*
    * Drivetrain
    */

    // Gear sensitivity, (2020 port)
    public static final double DT_HIGH_GEAR_SENSITIVITY = 0.35;
    public static final double DT_HIGH_GEAR_MOVING_SENSITIVITY = 0.45;
    public static final double DT_LOW_GEAR_SENSITIVITY = 0.625;

    // Hardware IDs, (2020 port)
    // Motors controlled by Spark Maxes
    public static final int DT_LEFT_MASTER = 1;
    public static final int DT_LEFT_SLAVE = 2;
    public static final int DT_LEFT_MASTER_ENC = 6; // A (2020 val)
    public static final int DT_LEFT_SLAVE_ENC = 7; // B

    public static final int DT_RIGHT_MASTER = 5;
    public static final int DT_RIGHT_SLAVE = 6;
    public static final int DT_RIGHT_MASTER_ENC = 4; // A
    public static final int DT_RIGHT_SLAVE_ENC = 5; // B

    public static final int DT_SHIFTER_FWD = 0;
    public static final int DT_SHIFTER_BCK = 7;

}