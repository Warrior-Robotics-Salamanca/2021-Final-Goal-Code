package org.firstinspires.ftc.teamcode.GameOpModes.CompetitionOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.CommandBuilders.AutonomousBuilder;
import org.firstinspires.ftc.teamcode.localizers.DeadWheelsAndIMULocalizer;
import org.firstinspires.ftc.teamcode.ringScanners.OpenCVRingScanner;
import org.firstinspires.ftc.teamcode.ringScanners.RingScanner;
import org.firstinspires.ftc.teamcode.ringScanners.TensorflowRingScanner;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.InterOpModeStorage;
import org.firstinspires.ftc.teamcode.util.Pose2D;

/**
 * The autonomous program for starting the robot on the right
 * start line.
 */
@Autonomous(name="Auto Right Line", group="Competition")
public class MainAutoRightLine extends LinearOpMode {
    private RobotContainer robot;

    private RingScanner ringScanner = null;

    private FtcDashboard dashboard;

    private CommandScheduler mainScheduler;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        //robot.localizer = new SwerveUnitAndIMULocalizer(robot.driveBase, robot.imu);
        robot.localizer = new DeadWheelsAndIMULocalizer(robot.xDeadWheel, robot.yDeadWheel, robot.imu);
        robot.localizer.setStartPose(new Pose2D(56.5 * DistanceUnit.mPerInch, -62 * DistanceUnit.mPerInch, Math.PI/2));
        robot.localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        robot.wobbleLifter.setClawClosed();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        mainScheduler = CommandScheduler.getInstance();

        telemetry.addLine("Hardware Initialized. Initializing ringScanner...");
        telemetry.update();

        ringScanner = new OpenCVRingScanner(hardwareMap);
        //dashboard.startCameraStream(((TensorflowRingScanner)ringScanner).getVuforia(), 0);
        //dashboard.startCameraStream(((TensorflowRingScanner)ringScanner).getTfod(), 0);

        SequentialCommandGroup zeroRingCommand = AutonomousBuilder.buildZeroRingAutoRightLine(robot);
        SequentialCommandGroup oneRingCommand = AutonomousBuilder.buildOneRingAutoRightLine(robot);
        SequentialCommandGroup fourRingCommand = AutonomousBuilder.buildFourRingAutoRightLine(robot);

        robot.intake.setRingBlockerUp();

        sleep(500);

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        robot.localizer.startLocalizer();

        int scannedRingCount = ringScanner.detectRingStackSize();

        telemetry.addData("Scanned Ring Count", scannedRingCount);
        telemetry.update();

        //scannedRingCount = 4; // override for testing
        if(scannedRingCount == 0) {
            mainScheduler.schedule(zeroRingCommand);
        } else if(scannedRingCount == 1) {
            mainScheduler.schedule(oneRingCommand);
        } else {
            mainScheduler.schedule(fourRingCommand);
        }

        while(opModeIsActive() && (!zeroRingCommand.isFinished() || !oneRingCommand.isFinished() || !fourRingCommand.isFinished())) {
            mainScheduler.run();

            // debug
            Pose2D robotPose = robot.localizer.getRobotPose();
            robot.driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY() + ",rot~" + robotPose.getTheta());
        }

        robot.intake.setRingBlockerDown();

        mainScheduler.disable();
        InterOpModeStorage.savedRobot = robot;
        ((OpenCVRingScanner)ringScanner).disable();
        robot.localizer.shutdown();
    }
}
