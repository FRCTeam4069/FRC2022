package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonGyro {
    
    PigeonIMU gyro;
    double atLastReset = 0;
    public PigeonGyro(PigeonIMU gyro) {
        this.gyro = gyro;
        
        atLastReset = gyro.getFusedHeading();
    }

    public double getCurrentHeading() {
        return gyro.getFusedHeading() - atLastReset;
    }

    public void resetHeading() {
        System.out.println("resetting");
        atLastReset = gyro.getFusedHeading();
    }


}
