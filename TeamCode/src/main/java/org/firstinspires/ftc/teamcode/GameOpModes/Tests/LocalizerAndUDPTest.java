package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.localizers.*;
import org.firstinspires.ftc.teamcode.network.UDPDriveStationClient;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorPlus;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.SwerveUnit;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Disabled
@TeleOp(name="Localizer/UDP test", group="Tests")
public class LocalizerAndUDPTest extends LinearOpMode {
    private Localizer localizerVelocProvider;
    private Localizer localizerPointMap;

    private SwerveDriveBase driveBase = null;

    private UDPDriveStationClient driveStation;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
            @Override
            public void uncaughtException(Thread t, Throwable e) {
                telemetry.addLine("Caught Exception in external thread: " + e);
                for(StackTraceElement elem : e.getStackTrace()) {
                    telemetry.addLine(elem.toString());
                }
                telemetry.update();
            }
        });

        driveStation = new UDPDriveStationClient("192.168.43.195", 50009);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initializeSwerveDrive(imu);

        /*DeadWheelEncoderUnit XUnit = new DeadWheelEncoderUnit(
                hardwareMap.get(DcMotorEx.class, "intakeMotor"),
                new Pose2D(.1, .2, 0),
                0.0508
        );

        DeadWheelEncoderUnit YUnit = new DeadWheelEncoderUnit(
                hardwareMap.get(DcMotorEx.class, "otherMotor"),
                new Pose2D(-.1, .2, 0),
                0.0508
        );*/

        DistanceSensorPlus distanceSensor1 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor1"),
                DriveBaseConstants.SENSOR1_OFFSET);
        DistanceSensorPlus distanceSensor2 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor2"),
                DriveBaseConstants.SENSOR2_OFFSET);
        DistanceSensorPlus distanceSensor3 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor3"),
                DriveBaseConstants.SENSOR3_OFFSET);

        //localizer = new SwerveUnitAndIMULocalizer(driveBase, imu);
        localizerVelocProvider = new SwerveUnitAndIMULocalizer(driveBase, imu);
        localizerVelocProvider.setStartPose(new Pose2D(0, 0, 0));
        localizerPointMap = new MonteCarloLocalizer(localizerVelocProvider, distanceSensor1, distanceSensor2, distanceSensor3);
        localizerPointMap.setOpModeRunningListener(new Button() {
            @Override
            public boolean get() {
                return opModeIsActive();
            }
        });

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        localizerPointMap.startLocalizer();

        sleep(200);

        telemetry.addLine("started!");
        telemetry.update();

        while(opModeIsActive()) {
            Pose2D robotPose = localizerPointMap.getRobotPose();;

            telemetry.addData("robotPose", robotPose.toString());
            telemetry.addData("poseError", ((MonteCarloLocalizer)localizerPointMap).getPoseError());
            telemetry.addData("sensor1 reading", distanceSensor1.sensor.getDistance(DistanceUnit.METER));
            telemetry.addData("sensor1 expected", distanceSensor1.simulateSensorReadingAtPose(robotPose));
            telemetry.addData("sensor2 reading", distanceSensor2.sensor.getDistance(DistanceUnit.METER));
            telemetry.addData("sensor2 expected", distanceSensor2.simulateSensorReadingAtPose(robotPose));
            telemetry.addData("sensor3 reading", distanceSensor3.sensor.getDistance(DistanceUnit.METER));
            telemetry.addData("sensor3 expected", distanceSensor3.simulateSensorReadingAtPose(robotPose));
            //telemetry.addData("leftAngle", driveBase.leftUnit.getSwivelAngle());
            //telemetry.addData("leftAngularVel", driveBase.leftUnit.getSwivelVelocity());
            telemetry.update();

            driveStation.sendStringUDP("x~" + robotPose.getX() + ",y~" + robotPose.getY() + ",rot~" + robotPose.getTheta());
        }
        localizerPointMap.shutdown();
        driveStation.sendStringUDP("disconnect");
    }


    /**
     * helper method to set up the swerve drive units
     */
    private void initializeSwerveDrive(BNO055IMU imu) {
        DcMotor leftTurnMotor = hardwareMap.get(DcMotor.class, "pink_turn");
        DcMotor leftDriveMotor = hardwareMap.get(DcMotor.class, "pink_drive");

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTurnMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SwerveUnit leftUnit = new SwerveUnit(DriveBaseConstants.leftWheelPosition,
                leftTurnMotor,
                leftDriveMotor);
        leftUnit.setSwerveTicksPerRevolution(1142);

        DcMotor rightTurnMotor = hardwareMap.get(DcMotor.class, "blue_turn");
        DcMotor rightDriveMotor = hardwareMap.get(DcMotor.class, "blue_drive");

        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SwerveUnit rightUnit = new SwerveUnit(DriveBaseConstants.rightWheelPosition,
                rightTurnMotor,
                rightDriveMotor);

        driveBase = new SwerveDriveBase(leftUnit, rightUnit);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }
}
