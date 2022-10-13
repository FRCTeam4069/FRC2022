package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Robot Indexer Subsystem */
public class Indexer {

    private static final int DRIVE_CAN = 21;
    private final CANSparkMax drive;
    private final edu.wpi.first.wpilibj.AnalogInput Sensor3;

    
    public Indexer() {
        drive = new CANSparkMax(DRIVE_CAN, MotorType.kBrushless);
         Sensor3 = new edu.wpi.first.wpilibj.AnalogInput(3);

    }
    /** Update the drive state of the indexer */
    public void drive(double speed) {
        drive.set(speed);
    }

    public boolean getSensor(){
        if(Sensor3.getVoltage() >= 1){  return true;}
        else return false;
    }
    

    public void driveWithSensor(){
        double timeInIndexer = 0;
        double amountOfShoots = 0;
        if(!getSensor()){ drive.set(-1);}
        else {
            while(getSensor()){
                timeInIndexer +=1;
                if(timeInIndexer >= 5){
                drive.set(0);
                if(getSensor() == false){
                    amountOfShoots +=1;
                    timeInIndexer =0;
                 }
                }
                drive.stopMotor();
            }
        }   



    }



}
