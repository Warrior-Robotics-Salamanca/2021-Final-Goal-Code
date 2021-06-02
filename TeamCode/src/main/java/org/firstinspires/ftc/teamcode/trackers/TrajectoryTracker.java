package org.firstinspires.ftc.teamcode.trackers;

import org.firstinspires.ftc.teamcode.util.CartesianVectorD;
import org.firstinspires.ftc.teamcode.util.PointPath;
import org.firstinspires.ftc.teamcode.util.RobotAction;
import org.firstinspires.ftc.teamcode.util.RobotState;
import org.firstinspires.ftc.teamcode.util.Trajectory;

public abstract class TrajectoryTracker {
    protected Trajectory path = null;

    public TrajectoryTracker() {
        path = new PointPath();
    }

    public TrajectoryTracker(Trajectory t) {
        path = t;
    }

    public void setTrajectory(Trajectory t) {
        path = t;
    }

    public abstract RobotAction updateAndGetAction(RobotState state);
    public abstract boolean isCompleted();
}
