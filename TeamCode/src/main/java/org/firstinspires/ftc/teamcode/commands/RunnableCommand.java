package org.firstinspires.ftc.teamcode.commands;

/**
 * A RunnableCommand is a command that runs
 * a single piece of code once, and then is
 * cancelled.
 */
public class RunnableCommand extends Command {

    private boolean isCancelled = false;
    private boolean isDone = false;

    private Runnable myRunnable = null;

    public RunnableCommand(Runnable runnable) {
        myRunnable = runnable;
    }

    @Override
    public void cancel() {
        isCancelled = true;
    }

    @Override
    public void execute() {
        myRunnable.run();
        isDone = true;
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
        return isCancelled || isDone;
    }
}
