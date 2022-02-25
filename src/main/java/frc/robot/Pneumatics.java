package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;

/** Pneumatics Controls Util */
public class Pneumatics {

    public static final int PN_PRESSURE_SENSOR = 0;

    private final AnalogInput pressureSensor;
    private final Compressor compressor;

    public Pneumatics() {
        pressureSensor = new AnalogInput(PN_PRESSURE_SENSOR);
        compressor = new Compressor(2, PneumaticsModuleType.REVPH);
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
