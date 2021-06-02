package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.util.CartesianVectorD;
import org.firstinspires.ftc.teamcode.util.PolarVectorD;

public class SwerveUnit {
    private Position unitPosition;
    private CartesianVectorD turnVector = new CartesianVectorD();
    private double distFromOrign;

    private DcMotorEx angleMotor, driveMotor;

    private double previousDistanceTraveled;

    private final double VELOCITY_DEADZONE = .01; // if velocity is 1% of max, stop spinning and turning

    private double ticksPerSwerveRevolution;

    public SwerveUnit(Position position, DcMotor angleMotor, DcMotor driveMotor) {
        unitPosition = position;
        PolarVectorD turnPolar = new PolarVectorD();
        turnPolar.fromCartesianVector(position.x, position.y);
        distFromOrign = turnPolar.radius;
        turnPolar.theta += Math.PI/2;
        turnVector.fromPolarVector(turnPolar);
        this.turnVector.normalize();
        this.angleMotor = (DcMotorEx) angleMotor;
        this.driveMotor = (DcMotorEx) driveMotor;
        ticksPerSwerveRevolution = DriveBaseConstants.TICKS_PER_SWERVE_UNIT_REVOLUTION;
    }

    /*
     * set the angular Velocity of the whole swerve unit (radians/sec)
     */
    public void setSwivelVelocity(double vel) {
        // TODO: make this more efficient
        angleMotor.setVelocity(vel * DriveBaseConstants.SWIVEL_GEAR_RATIO, AngleUnit.RADIANS);
    }

    /**
     * sets the exact angle to turn the swerve unit to
     * @param angle angle to set module heading to
     * @param turnUnit angle unit to use
     */
    public void setSwivelAngleAbsolute(double angle, AngleUnit turnUnit) {
        if(turnUnit.equals(AngleUnit.DEGREES)) angle = angle * Math.PI/180d;

        angleMotor.setTargetPosition((int) (angle * ticksPerSwerveRevolution/(2 * Math.PI)));
    }

    /**
     * finds the shortest arc to follow to get to the specified heading angle
     * @param angle angle to set module heading to
     * @param turnUnit angle unit to use
     */
    public void setSwivelAngleCircular(double angle, AngleUnit turnUnit) {
        if(turnUnit.equals(AngleUnit.DEGREES)) angle = angle * Math.PI/180d;

        double actualAngle = getSwivelAngle();
        while(actualAngle - angle >= Math.PI) angle += 2 * Math.PI;
        while(actualAngle - angle < -Math.PI) angle -= 2 * Math.PI;

        angleMotor.setTargetPosition((int) (angle * ticksPerSwerveRevolution/(2 * Math.PI)));
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(1);
    }

    public double getSwivelAngle() {
        return angleMotor.getCurrentPosition() * 2 * Math.PI / (ticksPerSwerveRevolution);
    }

    /**
     * vel in meters/second
     */
    public void setDriveVelocity(double vel) {
        driveMotor.setVelocity(((vel)/DriveBaseConstants.DISTANCE_PER_WHEEL_REVOLUTION + getSwivelVelocity()/(2*Math.PI)) * DriveBaseConstants.TICKS_PER_WHEEL_REVOLUTION);
    }

    /**
     *
     * @return velocity in meters/second
     */
    public double getDriveVelocity() {
        return (driveMotor.getVelocity() / DriveBaseConstants.TICKS_PER_WHEEL_REVOLUTION) * DriveBaseConstants.DISTANCE_PER_WHEEL_REVOLUTION;
    }

    /**
     *
     * @return velocity in radians/second
     */
    public double getSwivelVelocity() {
        return angleMotor.getVelocity() * 2 * Math.PI / (ticksPerSwerveRevolution);
    }

    public double getDistanceTraveledTotal() {
        return (driveMotor.getCurrentPosition() * DriveBaseConstants.WHEEL_REVOLUTIONS_PER_TICK) * DriveBaseConstants.DISTANCE_PER_WHEEL_REVOLUTION;
    }

    public void setUnitVelocity(Velocity vel) {
        double mag = Math.hypot(vel.xVeloc, vel.yVeloc);
        if(mag > VELOCITY_DEADZONE * DriveBaseConstants.MAX_DRIVE_VELOCITY) {
            setSwivelAngleCircular(Math.atan2(vel.yVeloc, vel.xVeloc), AngleUnit.RADIANS);
        }
        setDriveVelocity(mag);
    }

    public Velocity getTurnVelocityAtAngularVelocity(AngularVelocity angularVelocity) {
        //angularVelocity = angularVelocity.toAngleUnit(AngleUnit.RADIANS);
        CartesianVectorD temp = CartesianVectorD.mult(turnVector, distFromOrign * angularVelocity.zRotationRate);
        Velocity tempVel = new Velocity();
        tempVel.xVeloc = temp.x;
        tempVel.yVeloc = temp.y;
        return tempVel;
    }

    public void setDrivePower(double power) {
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setPower(power);
    }

    public void setSwerveTicksPerRevolution(double tpr) {
        ticksPerSwerveRevolution = tpr;
    }

    /**
     * use getDriveVelocity instead
     * @return dist traveled since last time this was called
     */
    @Deprecated
    public double getDistanceTraveledTimeStep() {
        double dist = getDistanceTraveledTotal();
        double distTraveled = dist - previousDistanceTraveled;
        previousDistanceTraveled = dist;
        return distTraveled;
    }
}
