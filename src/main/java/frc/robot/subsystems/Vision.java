package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.security.Timestamp;
import java.util.concurrent.atomic.DoubleAdder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    public NetworkTable table;

    public final double limelightMountingAngleDegs = 28;
    public final double limelightMountHeightInches = 30.0;
    public final double goalHeightInches = 101.5;

    private double lastReadDistance = 0;
    public double DistanceArray[] = new double[20];

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("").setNumber(1);
    }

    public void printDistanceToGoal() {
        table.getEntry("ledMode").setNumber(3);
        NetworkTableEntry ty = table.getEntry("ty");
        double verticalOffsetDegs = ty.getDouble(0.0);

        double angleToGoal = (verticalOffsetDegs + limelightMountingAngleDegs) * (3.1415926 / 180.0);
        double distance1 = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoal);
        double distance =  (0.946791 * distance1) - 4.80403;
        double realDistance =  0.0001496 * Math.pow(distance, 3) - 0.058627 * Math.pow(distance, 2) + 8.8964 * distance - 319.774;
        System.out.println("Distance to goal: " + realDistance);


        // lastReadDistance = realDistance;
    }


    public double getDistance() {
        NetworkTableEntry ty = table.getEntry("ty");
        double verticalOffsetDegs = ty.getDouble(0.0);

        double angleToGoal = (verticalOffsetDegs + limelightMountingAngleDegs) * (3.1415926 / 180.0);
        double distance1 = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoal);
        double distance = (0.946791 * distance1) - 4.80403;
        double realDistance =  0.0001496 * Math.pow(distance, 3) - 0.058627 * Math.pow(distance, 2) + 8.8964 * distance - 319.774;
        System.out.println("Distance to goal: " + realDistance);
        return realDistance;
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getLastReadDistance() {
        if(hasTarget()) lastReadDistance = getDistance();
        return lastReadDistance;
    }
    public void enableLED() {
        table.getEntry("ledMode").setNumber(3);
    }

    public void disableLED() {
        table.getEntry("ledMode").setNumber(1);
    }


    
}
