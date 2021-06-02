package org.firstinspires.ftc.teamcode.localizers;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.SwerveUnit;
import org.firstinspires.ftc.teamcode.util.CartesianVectorD;
import org.firstinspires.ftc.teamcode.util.PolarVectorD;
import org.firstinspires.ftc.teamcode.util.Pose2D;

public class SwerveUnitAndIMULocalizer extends Localizer {
    private Object lock = new Object(); // lock for pose synchronization across threads

    private double startingRot;
    private Pose2D globalPose;
    private Pose2D globalVelocity;

    private BNO055IMU imu;

    private SwerveDriveBase driveBase;

    private static final double SECONDS_PER_NANOSECOND = 1e-9;

    private long previousTime = 0;
    private double globalHeading = 0;
    private double heading = 0;
    private double previousHeading = 0;

    public SwerveUnitAndIMULocalizer(SwerveDriveBase driveBase, BNO055IMU imu) {
        this.driveBase = driveBase;
        this.imu = imu;
        setStartPose(new Pose2D(0, 0, 0));
        globalVelocity = new Pose2D(0, 0, 0);
    }

    @Override
    public Pose2D getRobotPose() {
        synchronized (lock) {
            return globalPose.copy();
        }
    }

    @Override
    public Pose2D getRobotVelocity() {
        synchronized (lock) {
            return globalVelocity.copy();
        }
    }

    @Override
    public void updatePose() {
        if(previousTime == 0) {
            previousTime = System.nanoTime();
            previousHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle + startingRot;
        }
        long timeLong = System.nanoTime();
        double dt = (timeLong - previousTime) * SECONDS_PER_NANOSECOND;
        previousTime = timeLong;

        heading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle + startingRot;
        double angleChange = (heading - previousHeading); // radians / sec
        if(angleChange >= Math.PI) {
            angleChange -= 2*Math.PI;
        } else if(angleChange <= -Math.PI) {
            angleChange += 2*Math.PI;
        }
        globalHeading += angleChange;
        previousHeading = heading;

        CartesianVectorD leftWheelVelocityVector = new CartesianVectorD(new PolarVectorD(driveBase.leftUnit.getSwivelAngle(), driveBase.leftUnit.getDriveVelocity()));
        CartesianVectorD rightWheelVelocityVector = new CartesianVectorD(new PolarVectorD(driveBase.rightUnit.getSwivelAngle(), driveBase.rightUnit.getDriveVelocity()));

        CartesianVectorD chassisVelocityLocal = leftWheelVelocityVector.add(rightWheelVelocityVector).mult(.5);

        synchronized (lock) {
            globalVelocity = new Pose2D(chassisVelocityLocal.x, chassisVelocityLocal.y, 0);
            Pose2D temp = globalVelocity.copy();
            temp.mult(dt);
            temp.rotateBy(globalHeading);
            globalVelocity.rotateBy(globalHeading);

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
