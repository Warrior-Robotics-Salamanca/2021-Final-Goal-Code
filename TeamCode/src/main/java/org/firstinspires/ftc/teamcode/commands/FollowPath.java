package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localizers.Localizer;
import org.firstinspires.ftc.teamcode.network.UDPDriveStationClient;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.trackers.PurePursuitTracker;
import org.firstinspires.ftc.teamcode.trackers.TrajectoryTracker;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.RobotAction;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.Trajectory;

/**
 * The FollowPath command accepts a path,
 * a localizing algorithm, and a tracking
 * algorithm to follow a path to completion
 */
public class FollowPath extends Command {
    private TrajectoryTracker tracker = null;
    private SwerveDriveBase driveBase = null;
    private Localizer localizer = null;

    private RobotState state = new RobotState();
    private RobotAction action = new RobotAction();

    private boolean isCancelled = false;

    public FollowPath(SwerveDriveBase driveBase, Localizer localizer, TrajectoryTracker tracker) {
        this.driveBase = driveBase;
        this.localizer = localizer;
        this.tracker = tracker;
    }

    @Override
    public void cancel() {
        isCancelled = true;
    }

    @Override
    public void execute() {
        Pose2D robotPose = localizer.getRobotPose();
        Pose2D robotVeloc = localizer.getRobotVelocity();
        state.xPosition = robotPose.getX();
        state.yPosition = robotPose.getY();
        state.xVelocity = robotVeloc.getX();
        state.yVelocity = robotVeloc.getY();
        state.rotation = robotPose.getTheta();

        action = tracker.updateAndGetAction(state);

        /*Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("action xveloc", action.xVelocity);
        telemetry.addData("action yveloc", action.yVelocity);
        telemetry.addData("action veloc mag", Math.hypot(action.yVelocity, action.xVelocity));
        telemetry.addData("action rotation", action.rotation);
        telemetry.addData("action angualrVeloc", action.angularVelocity);
        telemetry.update();*/

        driveBase.executeAction(action, state);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void end() {
        isScheduled = false;
    }

    @Override
    public boolean isFinished() {
        return isCancelled || tracker.isCompleted();
    }
}
