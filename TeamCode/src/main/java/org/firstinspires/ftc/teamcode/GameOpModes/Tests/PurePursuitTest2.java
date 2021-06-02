package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.trackers.PurePursuitTracker;
import org.firstinspires.ftc.teamcode.trackers.TrajectoryTracker;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.EndPose2D;
import org.firstinspires.ftc.teamcode.util.PointPath;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.Trajectory;

@Disabled
@TeleOp(group="Tests")
public class PurePursuitTest2 extends LinearOpMode {
    RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        robot.localizer = new SwerveUnitAndIMULocalizer(robot.driveBase, robot.imu);
        robot.localizer.setStartPose(new Pose2D(1.6001999, 0.0127, Math.PI/2));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        Trajectory path1 = new PointPath(new Pose2D[] {
                new Pose2D(1.6001999, 0.0127, Math.PI/2),
                new Pose2D(1.31445, 0.05715, Math.PI/2),
                new Pose2D(0.97789997, 0.26035, Math.PI/2),
                new Pose2D(0.7366, 0.4826, Math.PI/2),
                new Pose2D(0.6096, 0.69215, Math.PI/2),
                new Pose2D(0.53975, 0.9271, Math.PI/2),
                new Pose2D(0.51435, 1.1874499, Math.PI/2),
                new Pose2D(0.51435, 10, Math.PI/2)
        });

        Trajectory path2 = new PointPath();
        //path2.addPoint(new Pose2D(.6, 1, 0));
        //path2.addPoint(new EndPose2D(0, 0, 0));

        PurePursuitTracker tracker = new PurePursuitTracker(path1);
        tracker.setLookAheadDistance(.1, .1);
        tracker.setMaxVelocity(.4);

        telemetry.addLine("robotStatus: Initialized");
        telemetry.update();

        waitForStart();

        robot.localizer.startLocalizer();

        CommandScheduler.getInstance().schedule(new FollowPath(robot.driveBase, robot.localizer, tracker));

        while(opModeIsActive() && !tracker.isCompleted())
            CommandScheduler.getInstance().run();
        robot.driveBase.setVelocityToZero();
        sleep(5000);

        path2.addPoint(robot.localizer.getRobotPose());
        path2.addPoint(new EndPose2D(0.51435, 0, 0));

        tracker.setTrajectory(path2);
        PurePursuitTracker tracker2 = new PurePursuitTracker(path2);
        tracker2.setLookAheadDistance(.1, .1);
        tracker2.setMaxVelocity(.35);

        CommandScheduler.getInstance().schedule(new FollowPath(robot.driveBase, robot.localizer, tracker2));

        while(opModeIsActive() && !tracker2.isCompleted())
            CommandScheduler.getInstance().run();

        robot.driveBase.setVelocityToZero();
        robot.driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
        robot.driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);

        sleep(2000);

        robot.shutdown();
    }
}
