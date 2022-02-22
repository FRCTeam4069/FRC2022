package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

import static frc.robot.Constants.*;

public class Indexer implements RobotComponent {
    
    private final Robot robot;

    private CANSparkMax left, right;
    //private Encoder leftEncoder, rightEncoder;

    public Indexer(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {

        left = new CANSparkMax(ID_LEFT, MotorType.kBrushless);
        right = new CANSparkMax(ID_RIGHT, MotorType.kBrushless);

        return this;
    }

    @Override
    public void loop() {
        
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
