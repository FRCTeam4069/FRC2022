package frc.robot.auto.routines;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.auto.AutoScheduler;
import frc.robot.auto.commands.backIntake.DisableBackIntake;
import frc.robot.auto.commands.backIntake.EnableBakIntake;
import frc.robot.auto.commands.drivetrain.TrajectoryFollowerCommand;
import frc.robot.auto.commands.frontIntake.DisableIntakeCommand;
import frc.robot.auto.commands.frontIntake.EnableIntakeCommand;
import frc.robot.auto.commands.indexer.DisableIndexer;
import frc.robot.auto.commands.indexer.EnableIndexer;
import frc.robot.auto.commands.miscCommands.ResetGyroCommand;
import frc.robot.auto.commands.miscCommands.WaitCommand;
import frc.robot.auto.commands.shooter.PreFireShooterCommand;
import frc.robot.auto.commands.shooter.ShootCommand;
import frc.robot.auto.commands.shooter.ShootWithVisionCommand;
import frc.robot.auto.commands.vision.DisableLimelightLEDCommand;
import frc.robot.auto.commands.vision.EnableLimelightLEDCommand;

public class FourBallAuto implements AutoRoutine {

    private Robot robot;
    private AutoScheduler scheduler;

    public FourBallAuto(Robot robot) {
        this.robot = robot;
        scheduler = new AutoScheduler(robot);
    }

    @Override
    public String name() {
        return "Four Ball";
    }

    @Override
    public void init() {
        robot.getDriveTrain().setBrake();
        scheduler.addCommand(new ResetGyroCommand());
        scheduler.addCommand(new EnableBakIntake());
        scheduler.addCommand(new EnableIndexer(1));
        scheduler.addCommand(new PreFireShooterCommand(1300, 850)); //UNCOMMENT IF RED
       // scheduler.addCommand(new PreFireShooterCommand(1300, 595)); //UNCOMMENT IF BLUE
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        var start = robot.getDriveTrain().getPose();
        var end = new Pose2d(new Translation2d(-2.65, -0.2), new Rotation2d(Math.toRadians(12)));
        scheduler.addCommand(new TrajectoryFollowerCommand(start,
         interiorWaypoints, end, true));
    
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new EnableIntakeCommand());
        scheduler.addCommand(new EnableLimelightLEDCommand());
        scheduler.addCommand(new WaitCommand(0.25));
    
        scheduler.addCommand(new ShootWithVisionCommand(1300, 850, 3.5));// UNCOMMEBNT IF RED
       // scheduler.addCommand(new ShootCommand(1300, 595, 2.7)); // UNCOMMENT IF VLUE
        scheduler.addCommand(new DisableIntakeCommand());
        scheduler.addCommand(new DisableLimelightLEDCommand());
        scheduler.addCommand(new EnableBakIntake());
        scheduler.addCommand(new EnableIndexer(1));
        var backPickup = end.transformBy(new Transform2d(new Translation2d(-2.9, 1.7), new Rotation2d(Math.toRadians(25))));
        scheduler.addCommand(new TrajectoryFollowerCommand(end, interiorWaypoints, backPickup, true));

        var totalEnd = end.transformBy(new Transform2d(new Translation2d(0.4, 0), new Rotation2d(Math.toRadians(2))));
        scheduler.addCommand(new TrajectoryFollowerCommand(backPickup, interiorWaypoints, totalEnd, false));
        scheduler.addCommand(new DisableBackIntake());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new EnableIntakeCommand());
        scheduler.addCommand(new EnableLimelightLEDCommand());
        scheduler.addCommand(new PreFireShooterCommand(1300, 850)); //UNCOMMENT IF RED
       // scheduler.addCommand(new PreFireShooterCommand(1300, 600)); //UNCOMMENT IF BLUE
        scheduler.addCommand(new WaitCommand(0.25));
        scheduler.addCommand(new ShootWithVisionCommand(1300, 850, 3.5)); //UNCOMMENT IF RED
        //scheduler.addCommand(new ShootCommand(1300, 600, 3)); // UNCOMMENT IF BLUE
        scheduler.addCommand(new DisableIntakeCommand());
        scheduler.addCommand(new DisableLimelightLEDCommand());
        

        //PICKS UP BALL, JUST NEED TO DRIVE FORWARD AND SHOOT
    }

    @Override
    public void loop() {
        scheduler.run();
    }
    
}
