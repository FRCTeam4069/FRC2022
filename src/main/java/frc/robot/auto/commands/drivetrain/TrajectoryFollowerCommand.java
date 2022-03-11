package frc.robot.auto.commands.drivetrain;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.Command;
import frc.robot.subsystems.DriveTrain;

public class TrajectoryFollowerCommand extends Command {
    
    TrajectoryConfig config;
    Pose2d start;
    ArrayList<Translation2d> interiorWaypoints;
    Pose2d end;
    Trajectory trajectory;
    RamseteController controller;
    Pose2d currentPose;
    DifferentialDriveKinematics kinematics;
    private double trackWidth = 0.5; //meters

    public TrajectoryFollowerCommand(Pose2d start, ArrayList<Translation2d> interiorWaypoints, Pose2d end, boolean reversed) {
        
        config = new TrajectoryConfig(1, 0.25);
        config.setReversed(reversed);
        this.start = start;
        this.interiorWaypoints = interiorWaypoints;
        this.end = end;
        trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        controller = new RamseteController();
        kinematics = new DifferentialDriveKinematics(trackWidth);
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

        robot.getDriveTrain().updatePos();
        currentPose = robot.getDriveTrain().getPose();

        ChassisSpeeds speeds = controller.calculate(currentPose, trajectory.sample(elapsedTime));

        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        robot.getDriveTrain().updateDriveSpeeds(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
        robot.getDriveTrain().printImportantStuff();
    }
    @Override
    public boolean isFinished() {
        var poseDiff = robot.getDriveTrain().getPose().relativeTo(end);
        return (Math.abs(poseDiff.getX()) < 0.1) && (Math.abs(poseDiff.getY()) < 0.1); 
    }

    @Override
    public void close() {
        robot.getDriveTrain().setPower(0, 0);
    }
}
