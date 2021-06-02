package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.localizers.DeadWheelsAndIMULocalizer;
import org.firstinspires.ftc.teamcode.localizers.Localizer;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.network.UDPDriveStationClient;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.SwerveUnit;
import org.firstinspires.ftc.teamcode.trackers.PurePursuitTracker;
import org.firstinspires.ftc.teamcode.trackers.TrajectoryTracker;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.PointPath;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.RobotAction;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.Trajectory;

//@Disabled
@TeleOp(name="Pure Pursuit Controller test", group="Tests")
public class PurePursuit_Test extends LinearOpMode {
    private RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        robot.driveStation = new UDPDriveStationClient("192.168.43.173", 50009);

        Trajectory path1 = new PointPath();
        path1.addPoint(new Pose2D(0, 0, 0));
        path1.addPoint(new Pose2D(.6, 0, 0));
        path1.addPoint(new Pose2D(.6, 1, 0));

        Trajectory path2 = new PointPath();
        path2.addPoint(new Pose2D(.6, 1, 0));
        path2.addPoint(new Pose2D(0, 1, 0));
        path2.addPoint(new Pose2D(0, 0, 0));

        boolean path1Active = true;

        TrajectoryTracker tracker = new PurePursuitTracker(path1);
        ((PurePursuitTracker)tracker).setLookAheadDistance(.1, .1);

        robot.localizer = new DeadWheelsAndIMULocalizer(robot.xDeadWheel, robot.yDeadWheel, robot.imu);
        robot.localizer.setStartPose(new Pose2D(0, 0, 0));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        telemetry.addData("RobotStatus", "initialized and this is a test");
        telemetry.update();

        waitForStart();

        robot.localizer.startLocalizer();

        RobotState state = new RobotState();
        RobotAction action;
        while(opModeIsActive()) {
            CommandScheduler.getInstance().schedule(new FollowPath(robot.driveBase, robot.localizer, tracker));
            while (!tracker.isCompleted() && opModeIsActive()) {
                CommandScheduler.getInstance().run();

                Pose2D robotPose = robot.localizer.getRobotPose();
                /*Pose2D robotVeloc = robot.localizer.getRobotVelocity();
                state.xPosition = robotPose.getX();
                state.yPosition = robotPose.getY();
                state.xVelocity = robotVeloc.getX();
                state.yVelocity = robotVeloc.getY();
                state.rotation = robotPose.getTheta();

                action = tracker.updateAndGetAction(state);*/

                telemetry.addData("path1Active", path1Active);
                //telemetry.addData("xVeloc", action.xVelocity);
                //telemetry.addData("yVeloc", action.yVelocity);
                telemetry.update();

                //robot.driveBase.executeAction(action, state);
                robot.driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY() + ",rot~" + robotPose.getTheta());
            }
            path1Active = !path1Active;
            if(path1Active) {
                tracker.setTrajectory(path1);
            } else {
                tracker.setTrajectory(path2);
            }
        }

        robot.localizer.shutdown();
        robot.driveStation.sendStringUDP("disconnect");
    }
}
