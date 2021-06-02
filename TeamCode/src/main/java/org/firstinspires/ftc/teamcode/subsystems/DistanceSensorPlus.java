package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.util.CartesianVectorD;
import org.firstinspires.ftc.teamcode.util.Pose2D;

public class DistanceSensorPlus {
    public Pose2D offset = null; // position relative to center of robot.
    public DistanceSensor sensor = null;

    private Pose2D temp;

    public DistanceSensorPlus(DistanceSensor sensor, Pose2D offset) {
        this.sensor = sensor;
        this.offset = offset;
    }

    public double simulateSensorReadingAtPose(Pose2D robotPose) {
        Pose2D temp = new Pose2D(offset.getX(), offset.getY(), offset.getTheta());
        temp.rotateBy(robotPose.getTheta());
        temp.add(robotPose);

        while(temp.getTheta() < 0) temp.setTheta(temp.getTheta() + Math.PI * 2);
        temp.setTheta(temp.getTheta() % (Math.PI * 2));
        double slope = Math.tan(temp.getTheta());
        CartesianVectorD intersectionPoint = new CartesianVectorD();

        if (temp.getTheta() < Math.PI) {
            if (temp.getTheta() < Math.PI/2) {
                intersectionPoint.y = slope * (FieldConstants.CenterToWallWidth_M - temp.getX()) + temp.getY();
                if (intersectionPoint.y > FieldConstants.CenterToWallWidth_M) {
                    intersectionPoint.x = (FieldConstants.CenterToWallWidth_M - temp.getY()) / slope + temp.getX();
                    intersectionPoint.y = FieldConstants.CenterToWallWidth_M;
                } else {
                    intersectionPoint.x = FieldConstants.CenterToWallWidth_M;
                }
            } else {
                intersectionPoint.y = slope * (-FieldConstants.CenterToLeftWallWidth_M - temp.getX()) + temp.getY();
                if (intersectionPoint.y > FieldConstants.CenterToWallWidth_M) {
                    intersectionPoint.x = (FieldConstants.CenterToWallWidth_M - temp.getY()) / slope + temp.getX();
                    intersectionPoint.y = FieldConstants.CenterToWallWidth_M;
                } else {
                    intersectionPoint.x = -FieldConstants.CenterToLeftWallWidth_M;
                }
            }
        } else {
            if (temp.getTheta() > 3 * Math.PI/2) {
                intersectionPoint.y = slope * (FieldConstants.CenterToWallWidth_M - temp.getX()) + temp.getY();
                if (intersectionPoint.y < -FieldConstants.CenterToWallWidth_M) {
                    intersectionPoint.x = (-FieldConstants.CenterToWallWidth_M - temp.getY()) / slope + temp.getX();
                    intersectionPoint.y = -FieldConstants.CenterToWallWidth_M;
                } else {
                    intersectionPoint.x = FieldConstants.CenterToWallWidth_M;
                }
            } else {
                intersectionPoint.y = slope * (-FieldConstants.CenterToLeftWallWidth_M - temp.getX()) + temp.getY();
                if (intersectionPoint.y < -FieldConstants.CenterToWallWidth_M) {
                    intersectionPoint.x = (-FieldConstants.CenterToWallWidth_M - temp.getY()) / slope + temp.getX();
                    intersectionPoint.y = -FieldConstants.CenterToWallWidth_M;
                } else {
                    intersectionPoint.x = -FieldConstants.CenterToLeftWallWidth_M;
                }
            }
        }

        intersectionPoint.x -= temp.getX();
        intersectionPoint.y -= temp.getY();

        return Math.min(intersectionPoint.magnitude(), 2);
    }
}
