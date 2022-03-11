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
import frc.robot.auto.commands.shooter.ShootCommand;

public class TwoBallAuto implements AutoRoutine {

    private Robot robot;
    private AutoScheduler scheduler;

    public TwoBallAuto(Robot robot) {
        this.robot = robot;
        scheduler = new AutoScheduler(robot);
    }

    @Override
    public String name() {
        return "Two Ball";
    }

    @Override
    public void init() {
        scheduler.addCommand(new EnableBakIntake());
        scheduler.addCommand(new EnableIndexer(1));
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        var start = robot.getDriveTrain().getPose();
        var end = new Pose2d(new Translation2d(-2.5, 0), new Rotation2d(0));
        scheduler.addCommand(new TrajectoryFollowerCommand(start,
         interiorWaypoints, end, true));
        scheduler.addCommand(new DisableBackIntake());
        scheduler.addCommand(new DisableIndexer());
        var shootingPos = start.transformBy(new Transform2d(new Translation2d(-1, 0),
         new Rotation2d(Math.toRadians(40))));
        scheduler.addCommand(new TrajectoryFollowerCommand(end,
         new ArrayList<Translation2d>(), shootingPos, false));
        scheduler.addCommand(new EnableIntakeCommand());
        scheduler.addCommand(new ShootCommand(1300, 500, 5));
        scheduler.addCommand(new DisableIntakeCommand());
    }

    @Override
    public void loop() {
        scheduler.run();
    }
    
}
