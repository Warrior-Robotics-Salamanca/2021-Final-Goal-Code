package org.firstinspires.ftc.teamcode.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.subsystems.DeadWheelEncoderUnit;
import org.firstinspires.ftc.teamcode.util.Pose2D;

@Config
public class DeadWheelsAndIMULocalizer extends Localizer {

    Object lock = new Object();

    private double startingRot;

    private Pose2D globalPose;
    private Pose2D globalVeloc;

    private DeadWheelEncoderUnit XAxisEncoder;
    private DeadWheelEncoderUnit YAxisEncoder;
    private BNO055IMU imu;

    private final static double SECONDS_PER_NANOSECOND = 1e-9;

    public static double odoScale = 66;

    private double globalHeading;
    private long previousTime = 0;
    private double previousHeading = 0;
    private double previousXEncoderPos = 0;
    private double previousYEncoderPos = 0;
    private double currentXPos = 0;
    private double currentYPos = 0;

    private final double XEncoderDistFromCenter = Math.hypot(DriveBaseConstants.XDeadWheelPosition.getX(), DriveBaseConstants.XDeadWheelPosition.getY());
    private final double XEncoderAngleFromCenter = Math.atan2(DriveBaseConstants.XDeadWheelPosition.getY(), DriveBaseConstants.XDeadWheelPosition.getX());
    private final double YEncoderDistFromCenter = Math.hypot(DriveBaseConstants.YDeadWheelPosition.getX(), DriveBaseConstants.YDeadWheelPosition.getY());
    private final double YEncoderAngleFromCenter = Math.atan2(DriveBaseConstants.YDeadWheelPosition.getY(), DriveBaseConstants.YDeadWheelPosition.getX());

    public DeadWheelsAndIMULocalizer(DeadWheelEncoderUnit XAxisEncoder, DeadWheelEncoderUnit YAxisEncoder, BNO055IMU imu) {
        this.XAxisEncoder = XAxisEncoder;
        this.YAxisEncoder = YAxisEncoder;
        this.imu = imu;
        setStartPose(new Pose2D(0, 0, 0));
        globalVeloc = new Pose2D(0, 0, 0);
    }

    @Override
    public Pose2D getRobotPose() {
        synchronized (lock) {
            return new Pose2D(globalPose.getX(), globalPose.getY(), globalPose.getTheta());
        }
    }

    @Override
    public Pose2D getRobotVelocity() {
        synchronized (lock) {
            return globalVeloc.copy();
        }
    }

    @Override
    public void updatePose() {
        if(previousTime == 0) {
            previousTime = System.nanoTime();
            previousHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle + startingRot;
            previousXEncoderPos = XAxisEncoder.getPosition();
            previousYEncoderPos = YAxisEncoder.getPosition();
        }
        long timeLong = System.nanoTime();
        double dt = (timeLong - previousTime) * SECONDS_PER_NANOSECOND;
        previousTime = timeLong;

        // change which angle is retrieved based on expansion hub heading
        double heading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle + startingRot;
        double angleChange = (heading - previousHeading); // radians / sec
        if(angleChange >= Math.PI) {
            angleChange -= 2*Math.PI;
        } else if(angleChange <= -Math.PI) {
            angleChange += 2*Math.PI;
        }
        globalHeading += angleChange;
        previousHeading = heading;

        // calculate how encoders will rotate according to angular velocity
        double xEncoderVelocFromAngularVelocity = XEncoderDistFromCenter * -50 * angleChange  * Math.sin(XEncoderAngleFromCenter + XAxisEncoder.getRelativePose().getTheta());
        double yEncoderVelocFromAngularVelocity = YEncoderDistFromCenter * 50 * angleChange  * Math.sin(YEncoderAngleFromCenter + YAxisEncoder.getRelativePose().getTheta());

        synchronized (lock) {
            double localXVeloc = XAxisEncoder.getVelocity();
            double localYVeloc = YAxisEncoder.getVelocity();

            globalVeloc.setX(localXVeloc - xEncoderVelocFromAngularVelocity);
            globalVeloc.setY(localYVeloc - yEncoderVelocFromAngularVelocity);

            Pose2D temp = globalVeloc.copy();
            temp.mult(dt);
            temp.rotateBy(globalHeading);
            globalVeloc.rotateBy(globalHeading);

            globalPose.add(temp);
            globalPose.setTheta(globalHeading);
        }
    }

    @Override
    public void setStartPose(Pose2D pose) {
        globalHeading = pose.getTheta();
        globalPose = new Pose2D(pose.getX(), pose.getY(), globalHeading);
        startingRot = pose.getTheta();
    }
}
