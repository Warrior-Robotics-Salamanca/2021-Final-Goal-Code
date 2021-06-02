package org.firstinspires.ftc.teamcode.trackers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.util.EndPose2D;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.RobotAction;
import org.firstinspires.ftc.teamcode.util.CartesianVectorD;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.Trajectory;

@Config
public class PurePursuitTracker extends TrajectoryTracker {
    private double lookAheadVloc = .4; // placeholder for now. setLookAheadDistance() should be called instead
    private double lookAheadPath = .1; // placeholder for now. setLookAheadDistance() should be called instead

    private RobotAction myAction = new RobotAction(); // create this now so we dont continuously have to make an object
    private RobotState saveState;

    private final PIDController headingController; // for aligning to a target angle
    private final PIDController driveController;   // for slowing down when approaching end point

    private boolean targetFinalPoint = false;
    public static double endPositionTolerance = .09; // how close to the desired end position we are

    private double maxVel = DriveBaseConstants.MAX_DRIVE_VELOCITY;

    public PurePursuitTracker(Trajectory t) {
        this.setTrajectory(t);
        headingController = new PIDController(5.3, 0, 0);
        driveController   = new PIDController(DriveBaseConstants.DRIVE_ALIGN_KP, DriveBaseConstants.DRIVE_ALIGN_KI, DriveBaseConstants.HEADING_ALIGN_KD);
    }

    @Override
    public RobotAction updateAndGetAction(RobotState state) {
        //headingController.setPID(DriveBaseConstants.HEADING_ALIGN_KP, DriveBaseConstants.HEADING_ALIGN_KI, DriveBaseConstants.HEADING_ALIGN_KD);
        //driveController.setPID(DriveBaseConstants.DRIVE_ALIGN_KP, DriveBaseConstants.DRIVE_ALIGN_KI, DriveBaseConstants.DRIVE_ALIGN_KD);

        saveState = state;
        CartesianVectorD predict = new CartesianVectorD(state.xVelocity, state.yVelocity);

        // calculate desired drive velocity based on PID and maxVel
        double driveVelocity = maxVel;
        Pose2D endPoint = path.getPointAtIndex(path.size()-1);
        if(endPoint instanceof EndPose2D) {
            driveVelocity = Math.min(maxVel, -driveController.getOutput(Math.hypot(endPoint.getY() - state.yPosition, endPoint.getX() - state.xPosition), 0));
        }

        predict.normalize();
        predict.mult(lookAheadVloc);
        CartesianVectorD predictPoint = new CartesianVectorD(state.xPosition + predict.x, state.yPosition + predict.y);
        Pose2D pointOnPath = path.getClosestPointToPosition(predictPoint.x, predictPoint.y, lookAheadPath);
        double direction = Math.atan2(pointOnPath.getY() - state.yPosition, pointOnPath.getX() - state.xPosition);
        myAction.xVelocity = Math.cos(direction) * driveVelocity;
        myAction.yVelocity = Math.sin(direction) * driveVelocity;
        myAction.rotation = pointOnPath.getTheta();
        //myAction.angularVelocity = DriveBaseConstants.HEADING_ALIGN_KP * (myAction.rotation - state.rotation); // simple proportional rotation controller for now
        myAction.angularVelocity = headingController.getOutput(state.rotation, myAction.rotation); // compute correction
        //myAction.angularVelocity = 0;
        return myAction;
    }

    @Override
    public boolean isCompleted() {
        if(saveState == null) return false;
        Pose2D lastPoint = path.getPointAtIndex(path.size() - 1);
        return Math.hypot(saveState.yPosition - lastPoint.getY(), saveState.xPosition - lastPoint.getX()) < endPositionTolerance;
    }

    public void setLookAheadDistance(double lookAheadVeloc, double lookAheadPath) {
        this.lookAheadVloc = lookAheadVeloc;
        this.lookAheadPath = lookAheadPath;
    }

    public void setMaxVelocity(double maxVelocity) {
        this.maxVel = maxVelocity;
    }
}
