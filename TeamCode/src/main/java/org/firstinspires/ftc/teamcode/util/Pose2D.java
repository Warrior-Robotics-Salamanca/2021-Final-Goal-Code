package org.firstinspires.ftc.teamcode.util;

public class Pose2D {
    private double x;
    private double y;
    private double theta;

    public Pose2D(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }
    public void setX(double x) {this.x = x;}

    public double getY() {
        return y;
    }
    public void setY(double y) {this.y = y;}

    public double getTheta() {
        return theta;
    }
    public void setTheta(double angle) {theta = angle;}

    /**
     * rotate the x, y components
     * @param angle amount (radians) to rotate by
     */
    public void rotateBy(double angle) {
        double tempx = x * Math.cos(angle) - y * Math.sin(angle);
        this.y = x * Math.sin(angle) + y * Math.cos(angle);
        this.x = tempx;
    }

    public void add(Pose2D other) {
        this.x += other.getX();
        this.y += other.getY();
        this.theta += other.getTheta();
    }

    public void add(double x, double y, double theta) {
        this.x += x;
        this.y +=  y;
        this.theta += y;
    }

    public void mult(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
    }

    public Pose2D copy() {
        return new Pose2D(x, y, theta);
    }

    @Override
    public String toString() {
        return "pose = x; " + x + " y; " + y + " theta; " + theta;
    }
}
