package org.firstinspires.ftc.teamcode.GameOpModes.Tests.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.RunnableCommand;
import org.firstinspires.ftc.teamcode.commands.TargetPose;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.localizers.DeadWheelsAndIMULocalizer;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose2D;

//@Disabled
@Config
@TeleOp(group="Tuning")
public class TuneHeading extends LinearOpMode {
    private RobotContainer robot;

    private PIDController headingController = new PIDController(DriveBaseConstants.HEADING_ALIGN_KP,
            DriveBaseConstants.HEADING_ALIGN_KI,
            DriveBaseConstants.HEADING_ALIGN_KD);

    private Pose2D target = new Pose2D(100, 0, 0);

    public static double tuneMaxIOutput = .5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //telemetry.setAutoClear(false);

        //telemetry.addLine("Robot Initialized:");
        //telemetry.update();

        Telemetry.Item display1 = telemetry.addData("Correction", 0);
        Telemetry.Item display2 = telemetry.addData("targetAngle", 0);
        telemetry.update();

        waitForStart();

        robot.localizer = new DeadWheelsAndIMULocalizer(robot.xDeadWheel, robot.yDeadWheel, robot.imu);
        robot.localizer.setStartPose(new Pose2D(0, 0, Math.PI/2));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        robot.localizer.startLocalizer();

        /*Command leftTargeter = new TargetPose(robot.driveBase, robot.localizer, new Pose2D(-10, 0, 0), display1, display2);
        Command upTargeter = new TargetPose(robot.driveBase, robot.localizer, new Pose2D(0, 10, 0), display1, display2);
        Command rightTargeter = new TargetPose(robot.driveBase, robot.localizer, new Pose2D(10, 0, 0), display1, display2);

        Button targetLeftButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.x;
            }
        };
        targetLeftButton.onButtonPressed(leftTargeter);

        Button targetUpButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.y;
            }
        };
        targetUpButton.onButtonPressed(upTargeter);

        Button targetRightButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.b;
            }
        };
        targetRightButton.onButtonPressed(rightTargeter);

        boolean[] cancelAll = {false};
        Button KillAllCommandsButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.a;
            }
        };
        KillAllCommandsButton.onButtonPressed(new RunnableCommand(() -> {
            cancelAll[0] = true;
        }));*/

        //CommandScheduler.getInstance().addButtons(targetLeftButton, targetRightButton, targetUpButton, KillAllCommandsButton);

        while(opModeIsActive()) {
            /*CommandScheduler.getInstance().run();

            if(cancelAll[0]) {
                cancelAll[0] = false;
                CommandScheduler.getInstance().cancelAll();
                robot.driveBase.setVelocityToZero();
            }

            //telemetry.addData("leftCompleted", leftTargeter.isFinished());
            //telemetry.addData("upCompleted", upTargeter.isFinished());
            //telemetry.addData("rightCompleted", rightTargeter.isFinished());
            telemetry.update();
            */

            if(gamepad1.a) {
                target.setX(Math.cos(5*Math.PI/180)*100);
                target.setY(Math.sin(5*Math.PI/180)*100);
            } else if(gamepad1.b) {
                target.setY(100);
                target.setX(0);
            } else {
                target.setX(100);
                target.setY(0);
            }

            headingController.setPID(DriveBaseConstants.HEADING_ALIGN_KP,
                    DriveBaseConstants.HEADING_ALIGN_KI,
                    DriveBaseConstants.HEADING_ALIGN_KD);

            headingController.setMaxIOutput(tuneMaxIOutput);

            Pose2D robotPose = robot.localizer.getRobotPose();
            double targetHeading = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX())
                    + LauncherConstants.LAUNCH_OFFSET;

            while(robotPose.getTheta() - targetHeading >= Math.PI) targetHeading += 2 * Math.PI;
            while(robotPose.getTheta() - targetHeading < -Math.PI) targetHeading -= 2 * Math.PI;

            AngularVelocity correction = new AngularVelocity();
            correction.zRotationRate = (float)(headingController.getOutput(robotPose.getTheta(), targetHeading));
            //correction.zRotationRate = (float)((Math.PI/2 - 0));

            /*if(display1 != null && display2 != null) {
                display1.setCaption("Correction");
                display1.setValue((Math.PI/2 - 0));

                display1.setCaption("targetHeading");
                display1.setValue(targetHeading);
            }*/
            telemetry.addData("Correction", correction.zRotationRate);
            telemetry.addData("targetHeading", targetHeading * 180 / Math.PI);
            telemetry.addData("actualHeading", robotPose.getTheta() * 180/Math.PI);
            telemetry.addData("headingDifference", targetHeading * 180 / Math.PI - robotPose.getTheta() * 180/Math.PI);
            telemetry.update();

            Velocity vel = new Velocity();
            vel.xVeloc = 0; vel.yVeloc = 0;
            robot.driveBase.setPowerTankMode(vel, correction);
        }
        CommandScheduler.getInstance().disable();
        robot.shutdown();
    }
}
