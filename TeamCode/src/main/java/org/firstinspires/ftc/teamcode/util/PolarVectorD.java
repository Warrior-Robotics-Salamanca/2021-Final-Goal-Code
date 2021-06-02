package org.firstinspires.ftc.teamcode.util;

public class PolarVectorD {
    public double theta, radius;

    public PolarVectorD(double theta, double radius) {
        this.theta = theta;
        this.radius = radius;
    }

    public PolarVectorD() {
        this.theta = 0;
        this.radius = 0;
    }

    public PolarVectorD(CartesianVectorD vec) {
        fromCartesianVector(vec);
    }

    public PolarVectorD(PolarVectorD vec) {
        this.theta = vec.theta;
        this.radius = vec.radius;
    }

    public PolarVectorD normalize() {
        radius = 1;
        return this;
    }

    public PolarVectorD add(PolarVectorD other) {
        CartesianVectorD me = new CartesianVectorD(this);
        me = me.add(new CartesianVectorD(other));
        return new PolarVectorD(me);
    }

    public PolarVectorD fromCartesianVector(double x, double y) {
        this.theta = Math.atan2(y, x);
        this.radius = Math.hypot(x, y);
        return this;
    }

    public PolarVectorD fromCartesianVector(CartesianVectorD vec) {
        this.theta = Math.atan2(vec.y, vec.x);
        this.radius = Math.hypot(vec.x, vec.y);
        return this;
    }
}
