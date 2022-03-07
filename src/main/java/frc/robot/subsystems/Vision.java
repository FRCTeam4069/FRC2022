package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    NetworkTable table;

    public final double limelightMountingAngleDegs = 25.5;
    public final double limelightMountHeightInches = 30.0;
    public final double goalHeightInches = 101.5;

    public Vision() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("").setNumber(1);
    }

    public void printDistanceToGoal() {
        table.getEntry("ledMode").setNumber(3);
        NetworkTableEntry ty = table.getEntry("ty");
        double verticalOffsetDegs = ty.getDouble(0.0);

        double angleToGoal = (verticalOffsetDegs + limelightMountingAngleDegs) * (3.1415926 / 180.0);
        double distance = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoal);
        double realDistance = (1.48856 * distance) - 45.0587;
        System.out.println("Distance to goal: " + realDistance);
    }

    public double getDistance() {
        NetworkTableEntry ty = table.getEntry("ty");
        double verticalOffsetDegs = ty.getDouble(0.0);

        double angleToGoal = (verticalOffsetDegs + limelightMountingAngleDegs) * (3.1415926 / 180.0);
        double distance = (goalHeightInches - limelightMountHeightInches) / Math.tan(angleToGoal);
        double realDistance = (1.48856 * distance) - 45.0587;
        return realDistance;
    }

    public void enableLED() {
        table.getEntry("ledMode").setNumber(3);
    }

    public void disableLED() {
        table.getEntry("ledMode").setNumber(1);
    }

    
}
