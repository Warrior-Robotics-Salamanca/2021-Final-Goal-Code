package org.firstinspires.ftc.teamcode.localizers;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.Button;
import org.firstinspires.ftc.teamcode.util.Pose2D;

import java.util.function.Consumer;

public abstract class Localizer implements Runnable {
    protected boolean isRunning = false;
    protected Button opModeActive = null;

    public abstract Pose2D getRobotPose();
    public abstract Pose2D getRobotVelocity();
    public abstract void updatePose();
    public abstract void setStartPose(Pose2D pose);

    public void setOpModeRunningListener(Button listener) {
        opModeActive = listener;
    }

    @Override
    public void run() {
        if(opModeActive == null)
            throw new IllegalStateException("Must first call setOpModeRunningListener() and pass opModeIsActive() before running a localizer");
        while(isRunning && opModeActive.get()) {
            this.updatePose();
        }
    }

    public void shutdown() {
        isRunning = false;
    }

    public void startLocalizer() {
        if(!isRunning) {
            isRunning = true;
            new Thread(this).start();
        }
    }
}
