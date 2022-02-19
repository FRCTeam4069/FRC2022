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
        if (robot.getGamepad1().getBButton()){
            left.set(0.5);
            right.set(-0.5);
        } 
        else if(robot.getGamepad1().getXButton()) {
            left.set(-0.5);
            right.set(0.5);
        }
        else {
            left.set(0);
        }
    }
}
