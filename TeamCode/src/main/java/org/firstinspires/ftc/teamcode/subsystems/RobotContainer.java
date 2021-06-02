package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.localizers.Localizer;
import org.firstinspires.ftc.teamcode.network.UDPDriveStationClient;

public class RobotContainer {
    public Launcher launcher;
    public WobbleGoalLifter wobbleLifter;
    public IntakeAndTransfer intake;
    //public ScrimmageIntake scrimIntake;
    public SwerveDriveBase driveBase;
    public Localizer localizer;

    public DistanceSensorPlus distanceSensor1;
    public DistanceSensorPlus distanceSensor2;
    public DistanceSensorPlus distanceSensor3;
    public DistanceSensorPlus distanceSensor4;

    public BNO055IMU imu;

    public UDPDriveStationClient driveStation;

    private DcMotor rightTurnMotor;
    private DcMotor leftTurnMotor;
    private DcMotor rightDriveMotor;
    private DcMotor leftDriveMotor;

    public DeadWheelEncoderUnit xDeadWheel;
    public DeadWheelEncoderUnit yDeadWheel;

    public RobotContainer(OpMode opMode) {
        //CommandScheduler.getInstance().enable();
        initializeSubsystems(opMode);
    }

    public void initializeSubsystems(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        driveStation = new UDPDriveStationClient("192.168.43.173", 50009);

        // send all errors to this opMode's telemetry
        Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() {
            @Override
            public void uncaughtException(Thread t, Throwable e) {
                opMode.telemetry.addLine("Caught Exception in external thread" + t.toString() + ": " + e);
                for(StackTraceElement elem : e.getStackTrace()) {
                    opMode.telemetry.addLine(elem.toString());
                }
                opMode.telemetry.update();
            }
        });

        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 1.4, 0, 0)); // determined experimentally
        Servo ringIndexer = hardwareMap.get(Servo.class, "ring_indexer");
        launcher = new Launcher(launchMotor, ringIndexer);
        launcher.setIndexerHome();

        DcMotorEx lifterMotor = hardwareMap.get(DcMotorEx.class, "lifter_motor");
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo clawServo = hardwareMap.get(Servo.class, "claw");
        wobbleLifter = new WobbleGoalLifter(lifterMotor, clawServo);

        // attach intake motor to subsystem
        Servo transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        transferServo.setPosition(.5); // to stop it spinning on init
        intake = new IntakeAndTransfer(hardwareMap.get(DcMotor.class, "intake_motor"), hardwareMap.get(DcMotor.class, "transfer_motor"), hardwareMap.get(Servo.class, "ring_blocker"), transferServo);
        intake.setIntakePower(0);

        ////// TEMPORARY!!! //////
        //DcMotorEx intakeLifter = hardwareMap.get(DcMotorEx.class, "scrim_intake");
        //intakeLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Servo intakeServo = hardwareMap.get(Servo.class, "scrim_servo");
        //scrimIntake = new ScrimmageIntake(intakeLifter, intakeServo);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        distanceSensor1 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor1"),
                DriveBaseConstants.SENSOR1_OFFSET);
        distanceSensor2 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor2"),
                DriveBaseConstants.SENSOR2_OFFSET);
        distanceSensor3 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor3"),
               DriveBaseConstants.SENSOR3_OFFSET);
        //distanceSensor4 = new DistanceSensorPlus(hardwareMap.get(DistanceSensor.class, "sensor4"),
        //        DriveBaseConstants.SENSOR4_OFFSET);


        leftTurnMotor = hardwareMap.get(DcMotor.class, "blue_turn");
        leftDriveMotor = hardwareMap.get(DcMotor.class, "blue_drive");

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTurnMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SwerveUnit leftUnit = new SwerveUnit(DriveBaseConstants.leftWheelPosition,
                leftTurnMotor,
                leftDriveMotor);
        leftUnit.setSwerveTicksPerRevolution(1142);

        rightTurnMotor = hardwareMap.get(DcMotor.class, "pink_turn");
        rightDriveMotor = hardwareMap.get(DcMotor.class, "pink_drive");

        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTurnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurnMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        SwerveUnit rightUnit = new SwerveUnit(DriveBaseConstants.rightWheelPosition,
                rightTurnMotor,
                rightDriveMotor);
        rightUnit.setSwerveTicksPerRevolution(1142);

        driveBase = new SwerveDriveBase(leftUnit, rightUnit);

        DcMotorEx xDeadWheelMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        xDeadWheel = new DeadWheelEncoderUnit(xDeadWheelMotor, DriveBaseConstants.XDeadWheelPosition, DistanceUnit.mPerInch * DriveBaseConstants.DEAD_WHEEL_DIAMETER_IN);
        DcMotorEx yDeadWheelMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        yDeadWheel = new DeadWheelEncoderUnit(yDeadWheelMotor, DriveBaseConstants.YDeadWheelPosition, DistanceUnit.mPerInch * DriveBaseConstants.DEAD_WHEEL_DIAMETER_IN);

        // init imu
        while (!((LinearOpMode)opMode).isStopRequested() && !imu.isGyroCalibrated()) {
            ((LinearOpMode)opMode).sleep(50);
            ((LinearOpMode)opMode).idle();
        }
    }

    public void fixMotorsForTeleOp() {
        rightTurnMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTurnMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.launchMotor.setVelocityPIDFCoefficients(60, 1.4, 0, 0);
    }

    /**
     * shuts down all subsystems safely
     */
    public void shutdown() {
        CommandScheduler.getInstance().disable();
        driveBase.setVelocityToZero();
        localizer.shutdown();
        launcher.setLauncherPower(0);
        //intake.setIntakePower(0);
        driveStation.sendStringUDP("disconnect");
    }
}
