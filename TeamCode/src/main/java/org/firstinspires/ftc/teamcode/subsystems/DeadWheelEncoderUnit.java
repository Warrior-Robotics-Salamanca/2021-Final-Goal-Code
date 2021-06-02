package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.util.Pose2D;

public class DeadWheelEncoderUnit {
    private DcMotorEx encoder = null;
    private Pose2D myPose = null;
    private double wheelCircumfrence;

    private static final double SECONDS_PER_NANOSECOND = 1e-9;

    private long previousTime = 0;
    private double previousPos = 0;

    public DeadWheelEncoderUnit(DcMotorEx encoder, Pose2D relativePose, double wheelDiameter) {
        this.encoder = encoder;
        myPose = relativePose;
        wheelCircumfrence = wheelDiameter * Math.PI;
    }

    public double getPosition() {
        return ((double)encoder.getCurrentPosition() / DriveBaseConstants.DEAD_WHEEL_TICKS_PER_WHEEL_REVOLUTION) * wheelCircumfrence;
    }

    public double getVelocity() {
        if(previousTime == 0) {
            previousTime = System.nanoTime();
            previousPos = getPosition();
        }

        long timeLong = System.nanoTime();
        double dt = (timeLong - previousTime) * SECONDS_PER_NANOSECOND;
        previousTime = timeLong;

        double pos = getPosition();
        double ret = (pos - previousPos)/dt;
        previousPos = pos;

        return ret;
    }

    public Pose2D getRelativePose() {
        return myPose;
    }
}
