package frc.robot.auto.routines;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.auto.commands.miscCommands.WaitCommand;
import frc.robot.auto.commands.shooter.ShootCommand;

public class TwoBallLeft implements AutoRoutine {

    private Robot robot;
    private AutoScheduler scheduler;
    public TwoBallLeft(Robot robot) {
        this.robot = robot;
        scheduler = new AutoScheduler(robot);
    }

    @Override
    public String name() {
        // TODO Auto-generated method stub
        return "Left";
    }

    @Override
    public void init() {
        robot.getDriveTrain().setBrake();
        scheduler.addCommand(new EnableBakIntake());
        scheduler.addCommand(new EnableIndexer(1));
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
        var start = robot.getDriveTrain().getPose();
        var end = new Pose2d(new Translation2d(-2.7, -0.2), new Rotation2d(Math.toRadians(10)));
        scheduler.addCommand(new TrajectoryFollowerCommand(start, interiorWaypoints, end, true));
        scheduler.addCommand(new WaitCommand(0.25));
        scheduler.addCommand(new DisableBackIntake());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new EnableIntakeCommand());
        scheduler.addCommand(new ShootCommand(1300, 730, 5));
        scheduler.addCommand(new DisableIntakeCommand());
        scheduler.addCommand(new DisableIndexer());
        scheduler.addCommand(new DisableBackIntake());
    }

    @Override
    public void loop() {
        scheduler.run();
    }

    
}
