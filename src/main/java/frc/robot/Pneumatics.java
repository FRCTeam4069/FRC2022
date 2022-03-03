package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Scheduler.RobotRepeatingTask;

/** Pneumatics Controls Util */
public class Pneumatics {

    public static final int PRESSURE_SENSOR = 0;
    public static final int PNEUMATICS_CAN = 1;

    private final AnalogInput pressureSensor;
    private final Compressor compressor;

    /** Init */
    public Pneumatics(Robot robot) {
        pressureSensor = new AnalogInput(PRESSURE_SENSOR);
        compressor = new Compressor(PNEUMATICS_CAN, PneumaticsModuleType.REVPH);

        robot.getScheduler().schedule((RobotRepeatingTask) this::pneumaticsLoop);
    }

    private void pneumaticsLoop() {
        if (getPressure() < 100.0 && !compressor.enabled()) compressor.enableDigital();
        else if (getPressure() >= 110.0) compressor.disable();
    }

    /**
     * Gets the pressure of the compressor in PSI
     *  
     * @return PSI pressure
     */
    public double getPressure() {
        return 250.0 * (pressureSensor.getVoltage() / RobotController.getVoltage5V()) - 25.0;
    }  
    
}
