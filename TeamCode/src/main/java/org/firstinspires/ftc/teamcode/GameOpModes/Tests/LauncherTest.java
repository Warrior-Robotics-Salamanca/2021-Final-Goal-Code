package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.LaunchRings;
import org.firstinspires.ftc.teamcode.commands.TargetPose;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.localizers.Localizer;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.SwerveUnit;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Disabled
@TeleOp (name="Launcher Test", group="Tests")
public class LauncherTest extends LinearOpMode {
    private RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.launcher.setIndexerHome();

        waitForStart();

        robot.localizer = new SwerveUnitAndIMULocalizer(robot.driveBase, robot.imu);
        robot.localizer.setStartPose(new Pose2D(0.9144, 0.9144/3, 0));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });
        robot.localizer.startLocalizer();

        boolean prevButtonState = gamepad1.y;
        boolean launcherOnToggle = false;

        robot.launcher.setLauncherVelocity(3410);
        sleep(3000);
        Command ringLauncher = new LaunchRings(robot.launcher, 3);
        CommandScheduler.getInstance().schedule(ringLauncher);

        while(opModeIsActive() && !ringLauncher.isFinished()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("Current Velocity", robot.launcher.getLauncherVelocity());
            telemetry.addData("Target Velocity", 3410);
            telemetry.update();
        }

        while(opModeIsActive()) {
            CommandScheduler.getInstance().run();

            double batteryVoltage = 0;

            for(VoltageSensor v : hardwareMap.voltageSensor) {
                double voltage = v.getVoltage();
                if(voltage > batteryVoltage) {
                    batteryVoltage = voltage;
                }
            }

            if(gamepad1.y != prevButtonState && gamepad1.y) {
                launcherOnToggle = !launcherOnToggle;
            }
            prevButtonState = gamepad1.y;

            if(launcherOnToggle)
                robot.launcher.setPowerFromVoltage(batteryVoltage);
            else
                robot.launcher.setLauncherPower(0);

            if(gamepad1.a && launcherOnToggle) {
                for(int i = 0; i < 3; i++) {
                    robot.launcher.setIndexerLaunch();
                    sleep(LauncherConstants.INDEXER_MOVEMENT_DELAY);
                    robot.launcher.setIndexerHome();
                    sleep(Math.max(LauncherConstants.INDEXER_MOVEMENT_DELAY, LauncherConstants.LAUNCHER_SPEED_UP_TIME));
                }
            }

            if(gamepad1.b) {
                Command goalTargeter = new TargetPose(robot.driveBase, robot.localizer, FieldConstants.LAUNCH_GOAL_CENTER);
                CommandScheduler.getInstance().schedule(goalTargeter);
                while(!goalTargeter.isFinished() && opModeIsActive()) {

                }
                telemetry.addLine("pointing at the goal");
                telemetry.update();
            } else {
                telemetry.addData("robotHeading", robot.localizer.getRobotPose().getTheta());
                telemetry.update();
            }
        }
        robot.shutdown();
        CommandScheduler.getInstance().disable();
    }
}
