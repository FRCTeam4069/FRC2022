package frc.robot.components;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Pneumatics system controls
 */
public class Pneumatics implements RobotComponent {

    private AnalogInput pressureSensor = new AnalogInput(PN_PRESSURE_SENSOR);
    private Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
    
    @Override
    public RobotComponent init() {
        // TODO Auto-generated method stub
        return this;
    }

    @Override
    public void loop() {
        // TODO Auto-generated method stub
    }

    /**
     * Gets the pressure of the compressor in PSI
     *  
     * @return PSI pressure
     */
    public double getPressure() {
        // thank you Kaitlyn (2020 code)
        return 250.0 * (pressureSensor.getVoltage() / RobotController.getVoltage5V()) - 25.0;
    }  
    
}
