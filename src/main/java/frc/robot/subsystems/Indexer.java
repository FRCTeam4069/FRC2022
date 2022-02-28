package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Robot Indexer Subsystem */
public class Indexer {

    public static final int ID_LEFT = 21;
    public static final int ID_RIGHT = 20;

    private final CANSparkMax left, right;
    //private Encoder leftEncoder, rightEncoder;

    public Indexer() {
        left = new CANSparkMax(ID_LEFT, MotorType.kBrushless);
        right = new CANSparkMax(ID_RIGHT, MotorType.kBrushless);
    }

    public void update(boolean rotateOne, boolean rotateTwo) {
        if (rotateOne){
            left.set(1);
            right.set(-1);
        } 
        else if(rotateTwo) {
            left.set(-1);
            right.set(1);
        }
        else {
            left.set(0);
            right.set(0);
        }
    }
}
