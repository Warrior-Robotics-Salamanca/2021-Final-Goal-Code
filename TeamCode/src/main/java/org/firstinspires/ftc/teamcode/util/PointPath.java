package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

public class PointPath extends Trajectory {
    private ArrayList<Pose2D> allPoints = null;
    private boolean endPoseExists = false;

    public PointPath() {
        allPoints = new ArrayList<Pose2D>();
    }

    public PointPath(Pose2D[] initPoints) {
        allPoints = new ArrayList<Pose2D>();
        for(int i = 0; i < initPoints.length; i++) {
            //if(initPoints[i] instanceof EndPose2D) {
            //    endPoseExists = true;
            //    if(i != initPoints.length-1) throw new IllegalArgumentException("EndPose2D can only be used as the last point of a path");
            //}
            allPoints.add(initPoints[i]);
        }
    }

    @Override
    public Pose2D getPointAtTime(double time) {
        return new Pose2D(0, 0, 0);
    }

    @Override
    public Pose2D getClosestPointToPosition(double x, double y, double lookAhead) {
        double minLen = 10000000; // big number to start
        Pose2D closestPoint = getPointAtIndex(0);
        for(int i = 1; i < allPoints.size(); i++) {
            Pose2D a = allPoints.get(i-1);
            Pose2D b = allPoints.get(i);
            Pose2D normalPoint = getNormalPoint(x, y, a, b);
            Pose2D targetPoint;
            // limit
            if(normalPoint.getX() < Math.min(a.getX(), b.getX()) || normalPoint.getX() > Math.max(a.getX(), b.getX()) ||
                normalPoint.getY() < Math.min(a.getY(), b.getY()) || normalPoint.getY() > Math.max(a.getY(), b.getY())) {
                normalPoint = b;
                targetPoint = normalPoint;
            } else {
                CartesianVectorD dir = new CartesianVectorD(b.getX() - a.getX(), b.getY() - a.getY(), 0);
                dir.normalize();
                dir.mult(lookAhead);
                targetPoint = new Pose2D(normalPoint.getX() + dir.x, normalPoint.getY() + dir.y, normalPoint.getTheta());
            }

            double distBetweenPoints = Math.hypot(x - normalPoint.getX(), y - normalPoint.getY());
            if(distBetweenPoints <= minLen) {
                minLen = distBetweenPoints;
                closestPoint = targetPoint;
            }
        }
        return closestPoint;
    }

    @Override
    public Pose2D getClosestPointToPosition(Pose2D point, double lookAhead) {
        return null;
    }

    @Override
    public double rawDistanceOfPointFromEnd(Pose2D point) {
        Pose2D lastPoint = allPoints.get(allPoints.size()-1);
        return Math.hypot(point.getX() - lastPoint.getX(), point.getY() - lastPoint.getY());
    }

    @Override
    public void addPoint(Pose2D point) {
        if(point instanceof EndPose2D) endPoseExists = true;
        allPoints.add(point);
    }

    @Override
    public void addPoint(int index, Pose2D point) {
        if(point instanceof EndPose2D) endPoseExists = true;
        allPoints.add(index, point);
    }

    @Override
    public void removePoint(int index) {
        if(allPoints.get(index) instanceof EndPose2D) endPoseExists = false;
        allPoints.remove(index);
    }

    @Override
    public Pose2D getPointAtIndex(int index) throws IndexOutOfBoundsException {
        return allPoints.get(index);
    }

    @Override
    public int size() {
        return allPoints.size();
    }

    @Override
    public Pose2D getPointAtDistance(double distance) {
        return null;
    }

    @Override
    public double getDistanceToPoint(Pose2D point) {
        int index = getIndexClosestPointToPosition(point);
        return 0;
    }

    private int getIndexClosestPointToPosition(Pose2D point) {
        double minLen = 10000000; // big number to start
        int closestIndex = 0;
        for(int i = 1; i < allPoints.size(); i++) {
            Pose2D normalPoint = getNormalPoint(point.getX(), point.getY(), allPoints.get(i), allPoints.get(i-1));
            double distBetweenPoints = Math.hypot(point.getX() - normalPoint.getX(), point.getY() - normalPoint.getY());
            if(distBetweenPoints <= minLen) {
                minLen = distBetweenPoints;
                closestIndex = i - 1;
            }
        }
        return closestIndex;
    }

    /**
     * function that finds a point on a line segment normal to a point in space given 2 line segments.
     * Also linearly interpolates rotation between end points
     * @param px the x coordinate of the point in space to find the normal of on the line
     * @param py the y coordinate of the point in space to find the normal of on the line
     * @param a point a of the line segment
     * @param b point b of the line segment
     * @return
     */
    private Pose2D getNormalPoint(double px, double py, Pose2D a, Pose2D b) {
        CartesianVectorD ap = new CartesianVectorD(px - a.getX(), py - a.getY());
        CartesianVectorD ab = new CartesianVectorD(b.getX() - a.getX(), b.getY() - a.getY());
        ab.normalize();
        ab.mult(ap.dot(ab));
        double segmentLength = Math.hypot(a.getY() - b.getY(), a.getX() - b.getX());
        double subSegmentLength = Math.hypot(a.getY() - (a.getY() + ab.y), a.getX() - (a.getX() + ab.x));
        return new Pose2D(a.getX() + ab.x, a.getY() + ab.y, (subSegmentLength/segmentLength) * (b.getTheta()-a.getTheta()) + a.getTheta());
    }
}
