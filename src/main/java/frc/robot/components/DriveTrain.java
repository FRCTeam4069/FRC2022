package frc.robot.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import frc.robot.Robot;

import static frc.robot.Constants.*;

public class DriveTrain {
    
    private final Robot robot;

    private CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
    private Encoder leftEncoder, rightEncoder;
    private boolean highGear;

    /**
     * Initialize Drive Train
     * @param robot Active Robot instance
     */
    public DriveTrain(Robot robot) {

        // Dependency Injection
        this.robot = robot;

        // Motors (src: FRCTeam4069/Preseason2022/src/main/java/frc/robot/subsystems/Drivetrain.java#<init>)
        leftMaster = new CANSparkMax(DT_LEFT_MASTER, MotorType.kBrushless);
        leftSlave = new CANSparkMax(DT_LEFT_SLAVE, MotorType.kBrushless);
        rightMaster = new CANSparkMax(DT_RIGHT_MASTER, MotorType.kBrushless);
        rightSlave = new CANSparkMax(DT_RIGHT_SLAVE, MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftEncoder = new Encoder(DT_LEFT_MASTER_ENC, DT_LEFT_SLAVE_ENC, true, EncodingType.k1X);
        rightEncoder = new Encoder(DT_RIGHT_MASTER_ENC, DT_RIGHT_SLAVE_ENC, false, EncodingType.k1X);

        leftEncoder.reset();
        rightEncoder.reset();

        leftEncoder.setDistancePerPulse(1);
        rightEncoder.setDistancePerPulse(1);

        // TODO: Include shifter

        highGear = false;

    }

}