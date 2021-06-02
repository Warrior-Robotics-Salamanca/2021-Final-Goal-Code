package org.firstinspires.ftc.teamcode.util;

public class CartesianVectorD {
    public double x, y, z;

    public CartesianVectorD(double x, double y) {
        this.x = x;
        this.y = y;
        this.z = 0;
    }

    public CartesianVectorD(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public CartesianVectorD() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public CartesianVectorD(PolarVectorD vec) {
        fromPolarVector(vec);
    }

    public CartesianVectorD(CartesianVectorD vec) {
        this.x = vec.x;
        this.y = vec.y;
        this.z = vec.z;
    }

    /*
     * make vector magnitude 1
     */
    public CartesianVectorD normalize() {
        CartesianVectorD temp = new CartesianVectorD(x, y, z);
        temp.mult(1/Math.sqrt(temp.dot(this)));
        this.x = temp.x;
        this.y = temp.y;
        this.z = temp.z;
        return temp;
    }

    public CartesianVectorD mult(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
        return new CartesianVectorD(x, y, z);
    }

    public static CartesianVectorD mult(CartesianVectorD vec1, double scalar) {
        return new CartesianVectorD(vec1.x * scalar, vec1.y * scalar, vec1.z * scalar);
    }

    public CartesianVectorD add(CartesianVectorD other) {
        return new CartesianVectorD(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public double dot(CartesianVectorD dotVec) {
        return this.x * dotVec.x + this.y * dotVec.y + this.z * dotVec.z;
    }

    public double magnitude() {
        return Math.sqrt(this.dot(this));
    }

    public CartesianVectorD fromPolarVector(double theta, double radius) {
        this.x = Math.cos(theta) * radius;
        this.y = Math.sin(theta) * radius;
        this.z = 0;
        return this;
    }

    public CartesianVectorD fromPolarVector(PolarVectorD vec) {
        this.x = Math.cos(vec.theta) * vec.radius;
        this.y = Math.sin(vec.theta) * vec.radius;
        this.z = 0;
        return this;
    }
}
