package frc.robot.auto;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;

public class TrajectoryFollowerCommand extends Command {
    
    TrajectoryConfig config;
    Pose2d start;
    ArrayList<Translation2d> interiorWaypoints;
    Pose2d end;
    Trajectory trajectory;
    RamseteController controller;
    DriveTrain drivetrain;
    PigeonIMU gyro;
    Pose2d currentPose;

    public TrajectoryFollowerCommand(DriveTrain drivetrain, PigeonIMU gyro, Pose2d start, ArrayList<Translation2d> interiorWaypoints, Pose2d end) {
        
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        gyro.setFusedHeading(0);
        config = new TrajectoryConfig(5, 10);
        this.start = start;
        this.interiorWaypoints = interiorWaypoints;
        this.end = end;
        trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        controller = new RamseteController();
    }

    private double startTime = 0;
    private double elapsedTime = 0;
    
    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }

    double deltaX = 0;
    double deltaY = 0;

    @Override
    public void loop() {
        elapsedTime = Timer.getFPGATimestamp() - startTime;




        ChassisSpeeds speeds = controller.calculate(currentPose, trajectory.sample(elapsedTime));
        
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void close() {
        // TODO Auto-generated method stub
        
    }


}
