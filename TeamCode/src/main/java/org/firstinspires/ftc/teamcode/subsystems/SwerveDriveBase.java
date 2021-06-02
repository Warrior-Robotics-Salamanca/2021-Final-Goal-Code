package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.util.RobotAction;
import org.firstinspires.ftc.teamcode.util.RobotState;

public class SwerveDriveBase {
    public SwerveUnit leftUnit, rightUnit;

    private Velocity currentVelocity;

    private long previousTime;
    private final double SECONDS_PER_NANOSECOND = 1e-9;

    private boolean accelerationEnabled = false;

    public SwerveDriveBase(SwerveUnit leftSwerveUnit, SwerveUnit rightSwerveUnit) {
        this.leftUnit = leftSwerveUnit;
        this.rightUnit = rightSwerveUnit;
        currentVelocity = new Velocity();
        currentVelocity.xVeloc = 0;
        currentVelocity.yVeloc = 0;
    }

    /**
     * sets desired velocity and angular velocity relative to the robot's own heading
     * @param vel desired velocity, with the x-axis pointed towards the front of the robot
     * @param angularVelocity desired total angular velocity of robot
     */
    public void setVelocityLocal(Velocity vel, AngularVelocity angularVelocity) {
        long timeLong = System.nanoTime();
        double dt = (timeLong - previousTime) * SECONDS_PER_NANOSECOND;
        previousTime = timeLong;

        // implement acceleration
        if(accelerationEnabled) {
            double velAngle = Math.atan2(currentVelocity.yVeloc, currentVelocity.xVeloc);
            double temp = Math.abs(Math.cos(velAngle)) * DriveBaseConstants.MAX_DRIVE_ACCELERATION;
            if (vel.xVeloc > currentVelocity.xVeloc) {
                currentVelocity.xVeloc += temp * dt;
                if (vel.xVeloc < currentVelocity.xVeloc)
                    currentVelocity.xVeloc = vel.xVeloc;
            } else if (vel.xVeloc < currentVelocity.xVeloc) {
                currentVelocity.xVeloc -= temp * dt;
                if (vel.xVeloc > currentVelocity.xVeloc)
                    currentVelocity.xVeloc = vel.xVeloc;
            }

            temp = Math.abs(Math.sin(velAngle)) * DriveBaseConstants.MAX_DRIVE_ACCELERATION;
            if (vel.yVeloc > currentVelocity.yVeloc) {
                currentVelocity.yVeloc += temp * dt;
                if (vel.yVeloc < currentVelocity.yVeloc)
                    currentVelocity.yVeloc = vel.yVeloc;
            } else if (vel.yVeloc < currentVelocity.yVeloc) {
                currentVelocity.yVeloc -= temp * dt;
                if (vel.yVeloc > currentVelocity.yVeloc)
                    currentVelocity.yVeloc = vel.yVeloc;
            }
        } else {
            currentVelocity.xVeloc = vel.xVeloc;
            currentVelocity.yVeloc = vel.yVeloc;
        }

        // calculate drive velocities for each wheel
        Velocity leftVel = new Velocity();
        Velocity leftTurnVelocity = leftUnit.getTurnVelocityAtAngularVelocity(angularVelocity);
        leftVel.xVeloc = currentVelocity.xVeloc + leftTurnVelocity.xVeloc;
        leftVel.yVeloc = currentVelocity.yVeloc + leftTurnVelocity.yVeloc;
        double leftVelMagnitude = Math.hypot(leftVel.xVeloc, leftVel.yVeloc);

        Velocity rightVel = new Velocity();
        Velocity rightTurnVelocity = rightUnit.getTurnVelocityAtAngularVelocity(angularVelocity);
        rightVel.xVeloc = currentVelocity.xVeloc + rightTurnVelocity.xVeloc;
        rightVel.yVeloc = currentVelocity.yVeloc + rightTurnVelocity.yVeloc;
        double rightVelMagnitude = Math.hypot(rightVel.xVeloc, rightVel.yVeloc);

        // make sure to maintain ratio if velocity of either wheel goes over limit
        if(leftVelMagnitude > DriveBaseConstants.MAX_DRIVE_VELOCITY) {
            leftVel.xVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / leftVelMagnitude;
            leftVel.yVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / leftVelMagnitude;
            rightVel.xVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / leftVelMagnitude;
            rightVel.yVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / leftVelMagnitude;
        }
        if(rightVelMagnitude > DriveBaseConstants.MAX_DRIVE_VELOCITY) {
            leftVel.xVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / rightVelMagnitude;
            leftVel.yVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / rightVelMagnitude;
            rightVel.xVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / rightVelMagnitude;
            rightVel.yVeloc *= DriveBaseConstants.MAX_DRIVE_VELOCITY / rightVelMagnitude;
        }

        leftUnit.setUnitVelocity(leftVel);
        rightUnit.setUnitVelocity(rightVel);
    }

    /**
     * sets the desired velocity and angular velocity with respect to the field
     * @param vel desired velocity with respect to the field
     * @param angularVelocity desired total angular velocity of the robot
     * @param referenceAngle angle of the robot with respect to the field
     */
    public void setVelocityGlobal(Velocity vel, AngularVelocity angularVelocity, double referenceAngle) {
        referenceAngle = -referenceAngle;

        Velocity finalVel = new Velocity();
        // rotate the velocity vector by -referenceAngle
        finalVel.xVeloc = Math.cos(referenceAngle) * vel.xVeloc - Math.sin(referenceAngle) * vel.yVeloc;
        finalVel.yVeloc = Math.sin(referenceAngle) * vel.xVeloc + Math.cos(referenceAngle) * vel.yVeloc;

        setVelocityLocal(finalVel, angularVelocity);
    }

    /**
     * sets the desired velocity and angle with respect to the field
     * @param vel desired velocity with respect to the field
     * @param desiredAngle desired absolute angle of the robot with respect to the field
     * @param referenceAngle angle of the robot with respect to the field
     */
    public void setVelocityAndAngleGlobal(Velocity vel, double desiredAngle, double referenceAngle) {
        referenceAngle = -referenceAngle;

        Velocity finalVel = new Velocity();
        // rotate the velocity vector by -referenceAngle
        finalVel.xVeloc = Math.cos(referenceAngle) * vel.xVeloc - Math.sin(referenceAngle) * vel.yVeloc;
        finalVel.yVeloc = Math.sin(referenceAngle) * vel.xVeloc + Math.cos(referenceAngle) * vel.yVeloc;

        final double angleKP = .3;

        AngularVelocity a = new AngularVelocity();
        a.zRotationRate = (float)(angleKP * (desiredAngle - referenceAngle)); // simple proportional control for now
        setVelocityLocal(finalVel, a);
    }

    public void setVelocityTankMode(Velocity vel, AngularVelocity angularVelocity) {
        leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
        rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);

        rightUnit.setDriveVelocity(vel.xVeloc + angularVelocity.zRotationRate * DriveBaseConstants.rightWheelDistanceFromCenter);
        leftUnit.setDriveVelocity(vel.xVeloc - angularVelocity.zRotationRate * DriveBaseConstants.leftWheelDistanceFromCenter);
    }

    public void setPowerTankMode(Velocity vel, AngularVelocity angularVelocity) {
        leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
        rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);

        rightUnit.setDrivePower(vel.xVeloc + angularVelocity.zRotationRate);
        leftUnit.setDrivePower(vel.xVeloc - angularVelocity.zRotationRate);
    }

    public void executeAction(RobotAction action, RobotState state) {
        Velocity vel = new Velocity();
        vel.xVeloc = action.xVelocity;
        vel.yVeloc = action.yVelocity;
        AngularVelocity ang = new AngularVelocity();
        ang.zRotationRate = (float)action.angularVelocity;
        setVelocityGlobal(vel, ang, state.rotation);
    }

    public void setVelocityToZero() {
        leftUnit.setDrivePower(0);
        rightUnit.setDrivePower(0);
    }

    public void enableAcceleration() {
        accelerationEnabled = true;
    }

    public void disableAcceleration() {
        accelerationEnabled = false;
    }
}
