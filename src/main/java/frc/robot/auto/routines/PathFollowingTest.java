package frc.robot.auto.routines;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.auto.AutoScheduler;
import frc.robot.auto.commands.drivetrain.TrajectoryFollowerCommand;

public class PathFollowingTest implements AutoRoutine {

    Robot robot;
    AutoScheduler scheduler;
    public PathFollowingTest(Robot robot) {
        this.robot = robot;
        robot.getGyroscope().setFusedHeading(0);
    }

    @Override
    public String name() {
        return "Test Splines";
    }

    @Override
    public void init() {

        scheduler = new AutoScheduler(robot);
        robot.getDriveTrain().resetPos();
        var start = robot.getDriveTrain().getPose();
        var end = new Pose2d(new Translation2d(6, 1), new Rotation2d(0)); 
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        interiorWaypoints.add(new Translation2d(3, 0.5));
        scheduler.addCommand(new TrajectoryFollowerCommand(start, interiorWaypoints, end));   
    }

    @Override
    public void loop() {
        scheduler.run();
    }
    
}
