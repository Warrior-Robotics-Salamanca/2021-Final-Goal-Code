package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

public abstract class Trajectory {
     /**
     * return a point on the path at a given time
     * @param time time to get point at
     * @return
     */
    public abstract Pose2D getPointAtTime(double time);
    public abstract Pose2D getPointAtDistance(double distance);
    public abstract double getDistanceToPoint(Pose2D point);

    public abstract Pose2D getClosestPointToPosition(double x, double y, double lookAhead);
    public abstract Pose2D getClosestPointToPosition(Pose2D point, double lookAhead);

    public abstract double rawDistanceOfPointFromEnd(Pose2D point);

    public abstract void addPoint(Pose2D point);
    public abstract void addPoint(int index, Pose2D point);
    public abstract void removePoint(int index);
    public abstract Pose2D getPointAtIndex(int index) throws IndexOutOfBoundsException;
    public abstract int size();
}
