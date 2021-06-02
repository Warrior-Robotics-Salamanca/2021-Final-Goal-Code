package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * The LaunchRings command accepts a ring count
 * and an optional additional time delay to launch
 * rings with the indexer
 */
public class LaunchRings extends Command {

    private Launcher launcher;

    private boolean isCancelled = false;
    private boolean doneLaunching = false;

    private long speedUpDelay = 0;

    public enum IndexerState {
        MOVING_TO_LAUNCH,
        MOVING_TO_HOME
    };

    private IndexerState state = IndexerState.MOVING_TO_LAUNCH;
    private int ringsLeftToLaunch = 3;
    private int initRingsToLaunch = 3;

    private final double MS_PER_NS = 1e-6;

    private ElapsedTime timer;

    public LaunchRings(Launcher launcher, int ringCount) {
        this.launcher = launcher;
        initRingsToLaunch = Math.max(Math.min(ringCount, 3), 1);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public LaunchRings(Launcher launcher, int ringCount, long additionalSpeedUpDelay) {
        this.launcher = launcher;
        initRingsToLaunch = Math.max(Math.min(ringCount, 3), 1);
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.speedUpDelay = additionalSpeedUpDelay;
    }

    @Override
    public void cancel() {
        isCancelled = true;
    }

    @Override
    public void initialize() {
        //launcher.setIndexerHome();
        this.ringsLeftToLaunch = initRingsToLaunch;
        state = IndexerState.MOVING_TO_LAUNCH;
        timer.reset();
    }

    @Override
    public void execute() {
        double currentTimer_ms = timer.time();
        if(state.equals(IndexerState.MOVING_TO_LAUNCH)) {
            launcher.setIndexerLaunch();
            if(currentTimer_ms >= LauncherConstants.INDEXER_MOVEMENT_DELAY) {
                state = IndexerState.MOVING_TO_HOME;
                timer.reset();
            }
        } else if(state.equals(IndexerState.MOVING_TO_HOME)) {
            launcher.setIndexerHome();
            if(currentTimer_ms >= Math.max(LauncherConstants.INDEXER_MOVEMENT_DELAY, LauncherConstants.LAUNCHER_SPEED_UP_TIME) + speedUpDelay) {
                state = IndexerState.MOVING_TO_LAUNCH;
                timer.reset();
                if(--ringsLeftToLaunch == 0) doneLaunching = true;
            }
        }
    }

    @Override
    public void end() {
        isScheduled = false;
        isCancelled = false;
        doneLaunching = false;
    }

    @Override
    public boolean isFinished() {
        return isCancelled || doneLaunching;
    }
}
