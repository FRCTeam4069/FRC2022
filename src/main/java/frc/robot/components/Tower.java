package frc.robot.components;

import java.time.Duration;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalSource;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import frc.robot.Robot;

public class Tower implements RobotComponent {

    private static final int DT_RIGHT_SLAVE = 0;
    private static final int DT_RIGHT_MASTER = 0;
    private static final int DT_LEFT_MASTER = 0;
    private static final int DT_LEFT_SLAVE = 0;
    private static final DigitalSource DT_RIGHT_MASTER_ENC = null;
    private static final DigitalSource DT_LEFT_MASTER_ENC = null;
    private static final DigitalSource DT_LEFT_SLAVE_ENC = null;
    private static final DigitalSource DT_RIGHT_SLAVE_ENC = null;

    private Robot robot;
    private CANSparkMax leftMaster, leftSlave, rightMaster, rightSlave;
    private Encoder leftEncoder, rightEncoder;

    @Override
    public RobotComponent init() {

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

        // uses encoder to get RPM
        // RPM.setRate(RPM.getRate() * -1);

        // if (RPM.getCurrentTime().equals(Duration.ofMillis(0)) && RPM.getRate() == -1) {

        // }

        return this;

    }

    @Override
    public void shutdown() {

        leftMaster.close();
        leftSlave.close();
        rightMaster.close();
        rightSlave.close();
    }

    public void stop() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();

    }

    @Override
    public void loop() {
        

    }

}
