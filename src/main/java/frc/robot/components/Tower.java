package frc.robot.components;

import java.time.Duration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import frc.robot.Robot;

import static frc.robot.Constants.*;

public class Tower implements RobotComponent {
    
    private final Robot robot;

    private CANSparkMax left, right;
    //private Encoder leftEncoder, rightEncoder;

    public Tower(Robot robot) {
        this.robot = robot;
    }

    @Override
    public RobotComponent init() {

        left = new CANSparkMax(TW_LEFT, MotorType.kBrushless);
        right = new CANSparkMax(TW_RIGHT, MotorType.kBrushless);

        //leftEncoder = new Encoder(DT_LEFT_MASTER_ENC, DT_LEFT_SLAVE_ENC, true, EncodingType.k1X);
        //rightEncoder = new Encoder(DT_RIGHT_MASTER_ENC, DT_RIGHT_SLAVE_ENC, false, EncodingType.k1X);

        //leftEncoder.reset();
        //rightEncoder.reset(); 

        //leftEncoder.setDistancePerPulse(1);
        //rightEncoder.setDistancePerPulse(1);

        return this;
    }

    @Override
    public void loop() {
        if (robot.getGamepad1().getB()){
            left.set(ControlMode.PercentOutput, 1);
        } else {
            left.set(ControlMode.PercentOutput, 0);
        }
    }
}
