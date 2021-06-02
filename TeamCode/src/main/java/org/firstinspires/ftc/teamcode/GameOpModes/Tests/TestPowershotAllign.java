package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.LaunchRings;
import org.firstinspires.ftc.teamcode.commands.TargetPose;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@TeleOp
public class TestPowershotAllign extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotContainer robot = new RobotContainer(this);

        waitForStart();

        robot.localizer = new SwerveUnitAndIMULocalizer(robot.driveBase, robot.imu);
        robot.localizer.setStartPose(new Pose2D(24* DistanceUnit.mPerInch, - 24* DistanceUnit.mPerInch, 0));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        robot.localizer.startLocalizer();

        Pose2D robotPose = robot.localizer.getRobotPose();
        robot.launcher.setVelocityFromDistance(Math.hypot(robotPose.getY() - FieldConstants.LAUNCH_GOAL_CENTER.getY(), robotPose.getX() - FieldConstants.LAUNCH_GOAL_CENTER.getX()) / DistanceUnit.mPerInch, Launcher.LauncherTarget.POWERSHOT);
        sleep(2000);

        Command powershotTarget = new TargetPose(robot.driveBase, robot.localizer, FieldConstants.PEG3_CENTER, 3000)
                .andThen(new LaunchRings(robot.launcher, 1))
                .andThen(new TargetPose(robot.driveBase, robot.localizer, FieldConstants.PEG2_CENTER, 3000)
                        .andThen(new LaunchRings(robot.launcher, 1))
                        .andThen(new TargetPose(robot.driveBase, robot.localizer, FieldConstants.PEG1_CENTER, 3000)
                                .andThen(new LaunchRings(robot.launcher, 1))));

        CommandScheduler.getInstance().schedule(powershotTarget);

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }

        CommandScheduler.getInstance().disable();
        robot.shutdown();
    }
}
