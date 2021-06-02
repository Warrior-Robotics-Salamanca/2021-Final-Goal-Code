package org.firstinspires.ftc.teamcode.GameOpModes.Tests.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.localizers.DeadWheelsAndIMULocalizer;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.trackers.PurePursuitTracker;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.EndPose2D;
import org.firstinspires.ftc.teamcode.util.PointPath;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.Trajectory;
import org.opencv.core.Mat;

@Disabled
@TeleOp(group="Tuning")
public class TuneOdometry extends LinearOpMode {
    RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        robot.localizer = new DeadWheelsAndIMULocalizer(robot.xDeadWheel, robot.yDeadWheel, robot.imu);
        robot.localizer.setStartPose(new Pose2D(24* DistanceUnit.mPerInch, -2*24*DistanceUnit.mPerInch, Math.PI/2));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        Trajectory path1 = new PointPath(new Pose2D[] {
                new Pose2D(24* DistanceUnit.mPerInch, -2*24*DistanceUnit.mPerInch, Math.PI/2),
                new EndPose2D(24* DistanceUnit.mPerInch, 0* DistanceUnit.mPerInch, Math.PI/2)
        });

        Trajectory path2 = new PointPath();
        path2.addPoint(robot.localizer.getRobotPose());
        path2.addPoint(new Pose2D(24* DistanceUnit.mPerInch, -2*24*DistanceUnit.mPerInch, Math.PI/2));
        //path2.addPoint(new Pose2D(.6, 1, 0));
        //path2.addPoint(new EndPose2D(0, 0, 0));

        PurePursuitTracker tracker = new PurePursuitTracker(path1);
        tracker.setLookAheadDistance(.1, .1);
        tracker.setMaxVelocity(.4);

        telemetry.addLine("robotStatus: Initialized");
        telemetry.update();

        waitForStart();

        robot.localizer.startLocalizer();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                CommandScheduler.getInstance().schedule(new FollowPath(robot.driveBase, robot.localizer, tracker));

                while (opModeIsActive() && !tracker.isCompleted()) {
                    Pose2D robotPose = robot.localizer.getRobotPose();
                    robot.driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY() + ",rot~" + robotPose.getTheta());
                    CommandScheduler.getInstance().run();
                }
                robot.driveBase.setVelocityToZero();
                sleep(5000);

                tracker.setTrajectory(path2);
                PurePursuitTracker tracker2 = new PurePursuitTracker(path2);
                tracker2.setLookAheadDistance(.1, .1);
                tracker2.setMaxVelocity(.35);

                CommandScheduler.getInstance().schedule(new FollowPath(robot.driveBase, robot.localizer, tracker2));

                while (opModeIsActive() && !tracker2.isCompleted()) {
                    Pose2D robotPose = robot.localizer.getRobotPose();
                    robot.driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY() + ",rot~" + robotPose.getTheta());
                    CommandScheduler.getInstance().run();
                }

                robot.driveBase.setVelocityToZero();
                robot.driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                robot.driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
            }
        }

        robot.shutdown();
    }
}
