package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/** Climber subsystem */
public class Climber {

    // UPDATE CAN IDs!
    private static final int LEFT = -1;
    private static final int LEFT_ENC_A = 0;
    private static final int LEFT_ENC_B = 1;

    private static final int RIGHT = -1;
    private static final int RIGHT_ENC_A = 2;
    private static final int RIGHT_ENC_B = 3;

    private final TalonFX left, right;
    private final Encoder leftEncoder, rightEncoder;

    public Climber() {
        this.left = new TalonFX(LEFT);
        this.right = new TalonFX(RIGHT);

        leftEncoder = new Encoder(LEFT_ENC_A, LEFT_ENC_B, true, EncodingType.k1X);
        rightEncoder = new Encoder(RIGHT_ENC_A, RIGHT_ENC_B, false, EncodingType.k1X);
    }

}


