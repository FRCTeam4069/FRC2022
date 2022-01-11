package frc.robot.components;

import edu.wpi.first.wpilibj.Spark;

import static frc.robot.Constants.*;

public class Drivetrain {
    
    Spark leftMaster = new Spark(DT_LEFT_MASTER),
        leftSlave = new Spark(DT_LEFT_SLAVE),
        rightMaster = new Spark(DT_RIGHT_MASTER),
        rightSlave = new Spark(DT_RIGHT_SLAVE);

}
