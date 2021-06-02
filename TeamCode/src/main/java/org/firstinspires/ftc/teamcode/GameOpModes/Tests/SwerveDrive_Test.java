package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.localizers.Localizer;
import org.firstinspires.ftc.teamcode.localizers.SwerveUnitAndIMULocalizer;
import org.firstinspires.ftc.teamcode.network.UDPDriveStationClient;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.SwerveUnit;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import com.acmerobotics.dashboard.config.Config;

@Disabled
@TeleOp(name="Swerve Drive Test", group="Tests")
@Config
public class SwerveDrive_Test extends LinearOpMode {
    private Launcher launcher = null;

    private SwerveDriveBase driveBase = null;

    private Localizer localizer = null;

    private UDPDriveStationClient driveStation;

    public static int test = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        driveStation = new UDPDriveStationClient("192.168.43.195", 50009);

        DcMotor launchMotor = hardwareMap.get(DcMotor.class, "launch_motor");
        Servo ringIndexer = hardwareMap.get(Servo.class, "ring_indexer");

        launcher = new Launcher(launchMotor, ringIndexer);

        double desiredAngle = 0;
        double previousTime;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        DcMotor leftTurnMotor = hardwareMap.get(DcMotor.class, "blue_turn");
        DcMotor leftDriveMotor = hardwareMap.get(DcMotor.class, "blue_drive");

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTurnMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        SwerveUnit leftUnit = new SwerveUnit(DriveBaseConstants.leftWheelPosition,
                leftTurnMotor,
                leftDriveMotor);

        DcMotor rightTurnMotor = hardwareMap.get(DcMotor.class, "pink_turn");
        DcMotor rightDriveMotor = hardwareMap.get(DcMotor.class, "pink_drive");

        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurnMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SwerveUnit rightUnit = new SwerveUnit(DriveBaseConstants.rightWheelPosition,
                rightTurnMotor,
                rightDriveMotor);
        rightUnit.setSwerveTicksPerRevolution(1142);

        driveBase = new SwerveDriveBase(leftUnit, rightUnit);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addLine("Robot Initialized:");
        telemetry.update();

        waitForStart();

        localizer = new SwerveUnitAndIMULocalizer(driveBase, imu);
        localizer.setStartPose(new Pose2D(0, 0, Math.PI/2));
        localizer.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        localizer.startLocalizer();

        previousTime = getRuntime();
        boolean isTankDrive = false;
        boolean prevGamA = gamepad1.a;
        boolean prevGamY = gamepad1.y;
        boolean launcherOnToggle = false;

        while(opModeIsActive()) {
            double thisTime = getRuntime();
            double dt = thisTime - previousTime;
            previousTime = thisTime;

            double batteryVoltage = 0;

            for(VoltageSensor v : hardwareMap.voltageSensor) {
                double voltage = v.getVoltage();
                if(voltage > batteryVoltage) {
                    batteryVoltage = voltage;
                }
            }

            if(gamepad1.y != prevGamY && gamepad1.y) {
                launcherOnToggle = !launcherOnToggle;
            }
            prevGamY = gamepad1.y;

            if(launcherOnToggle)
                launcher.setPowerFromVoltage(batteryVoltage);
            else
                launcher.setLauncherPower(0);

            if(gamepad1.x && launcherOnToggle) {
                for(int i = 0; i < 3; i++) {
                    launcher.setIndexerLaunch();
                    sleep(LauncherConstants.INDEXER_MOVEMENT_DELAY);
                    launcher.setIndexerHome();
                    sleep(Math.max(LauncherConstants.INDEXER_MOVEMENT_DELAY, LauncherConstants.LAUNCHER_SPEED_UP_TIME));
                }
            }

            Pose2D robotPose = localizer.getRobotPose();
            double heading = robotPose.getTheta();

            if(!isTankDrive) {
                Velocity desiredVelocity = new Velocity();
                desiredVelocity.xVeloc = gamepad1.left_stick_x * DriveBaseConstants.MAX_DRIVE_VELOCITY;
                desiredVelocity.yVeloc = -gamepad1.left_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY;
                AngularVelocity desiredAngularVelocity = new AngularVelocity();
                desiredAngularVelocity.zRotationRate = -gamepad1.right_stick_x * (float) Math.PI; // turning speed in rad/sec
                //desiredAngularVelocity.zRotationRate = 1;
                driveBase.setVelocityGlobal(desiredVelocity, desiredAngularVelocity, heading);
                //desiredAngle += -(gamepad1.right_stick_x * Math.PI/4) * dt;
                //driveBase.setVelocityAndAngleGlobal(desiredVelocity, desiredAngle, heading);

                //leftUnit.setUnitVelocity(desiredVelocity);
                //leftUnit.setDriveVelocity(desiredVelocity.xVeloc);
                //leftUnit.setSwivelVelocity(desiredVelocity.yVeloc);
            } else {
                /*Velocity driveVelocity = new Velocity();
                driveVelocity.xVeloc = -gamepad1.left_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY;
                AngularVelocity desiredAngularVelocity = new AngularVelocity();
                desiredAngularVelocity.zRotationRate = -gamepad1.right_stick_x * (float) Math.PI; // turning speed in rad/sec
                driveBase.setVelocityTankMode(driveVelocity, desiredAngularVelocity);*/
                driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
                driveBase.leftUnit.setDriveVelocity(-gamepad1.left_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY);
                driveBase.rightUnit.setDriveVelocity(-gamepad1.right_stick_y * DriveBaseConstants.MAX_DRIVE_VELOCITY);
            }

            telemetry.addData("rightWheelVelocity", driveBase.rightUnit.getDriveVelocity());


            if(gamepad1.a && !prevGamA) {
                isTankDrive = !isTankDrive;
            }
            prevGamA = gamepad1.a;

            telemetry.addData("joyLeftX", gamepad1.left_stick_x);
            telemetry.addData("joyLeftY", gamepad1.left_stick_y);
            telemetry.addData("joyRightX", gamepad1.right_stick_x);
            telemetry.update();

            driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY() + ",rot~" + robotPose.getTheta());
        }
        localizer.shutdown();
        driveStation.sendStringUDP("disconnect");
    }
}
