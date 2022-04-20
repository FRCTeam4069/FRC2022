package frc.robot.auto.routines;

import java.util.ArrayList;

import javax.swing.ToolTipManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.auto.AutoScheduler;
import frc.robot.auto.commands.backIntake.DisableBackIntake;
import frc.robot.auto.commands.backIntake.EnableBakIntake;
import frc.robot.auto.commands.drivetrain.AutoAlignCommand;
import frc.robot.auto.commands.drivetrain.TrajectoryFollowerCommand;
import frc.robot.auto.commands.frontIntake.DisableIntakeCommand;
import frc.robot.auto.commands.frontIntake.EnableIntakeCommand;
import frc.robot.auto.commands.indexer.DisableIndexer;
import frc.robot.auto.commands.indexer.EnableIndexer;
import frc.robot.auto.commands.miscCommands.ResetGyroCommand;
import frc.robot.auto.commands.miscCommands.WaitCommand;
import frc.robot.auto.commands.shooter.PreFireShooterCommand;
import frc.robot.auto.commands.shooter.ShootWithVisionCommand;
import frc.robot.auto.commands.vision.DisableLimelightLEDCommand;
import frc.robot.auto.commands.vision.EnableLimelightLEDCommand;

public class NewFourBall implements AutoRoutine {
    private Robot robot;
    private AutoScheduler scheduler;

    public NewFourBall(Robot robot) {
        this.robot = robot;
        scheduler = new AutoScheduler(robot);
    }

    @Override
    public String name() {
        return "Two Ball";
    }

    @Override
    public void init() {
        robot.getDriveTrain().setBrake();
        scheduler.addCommand(new ResetGyroCommand());
        scheduler.addCommand(new EnableBakIntake());
        scheduler.addCommand(new EnableIndexer(1));
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        var start = robot.getDriveTrain().getPose();
        var end = new Pose2d(new Translation2d(-2.5, 0), new Rotation2d(Math.toRadians(0)));
        // scheduler.addCommand(new PreFireShooterCommand(1300, 650));
        scheduler.addCommand(new TrajectoryFollowerCommand(start, interiorWaypoints, end, true));
        scheduler.addCommand(new EnableLimelightLEDCommand());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new EnableIndexer(1));
        scheduler.addCommand(new WaitCommand(0.25));
        scheduler.addCommand(new DisableBackIntake());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new EnableIntakeCommand());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new PreFireShooterCommand(1300, 850));
        scheduler.addCommand(new AutoAlignCommand());
        scheduler.addCommand(new ShootWithVisionCommand(1300, 850, 3.5));
        scheduler.addCommand(new DisableIntakeCommand());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new DisableBackIntake());
        scheduler.addCommand(new DisableLimelightLEDCommand());
        scheduler.addCommand(new ResetGyroCommand());
        var backPickup = end.transformBy(new Transform2d(new Translation2d(-3.15, 1.1), new Rotation2d(Math.toRadians(0))));
        scheduler.addCommand(new EnableBakIntake());
        scheduler.addCommand(new EnableIndexer(1));
        scheduler.addCommand(new TrajectoryFollowerCommand(end, interiorWaypoints, backPickup, true));

        var totalEnd = end.transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.toRadians(10))));
        scheduler.addCommand(new TrajectoryFollowerCommand(backPickup, interiorWaypoints, totalEnd, false));
        scheduler.addCommand(new DisableBackIntake());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new PreFireShooterCommand(1300, 900));
        scheduler.addCommand(new AutoAlignCommand());
        scheduler.addCommand(new EnableIntakeCommand());
        scheduler.addCommand(new ShootWithVisionCommand(1300, 900, 3));
        scheduler.addCommand(new DisableIntakeCommand());
    }

    @Override
    public void loop() {
        scheduler.run();
    }
}
