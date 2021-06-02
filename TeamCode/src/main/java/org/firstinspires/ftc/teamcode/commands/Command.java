package org.firstinspires.ftc.teamcode.commands;

/**
 * The Command class specifies what functions a command must
 * implement in order to be executed by the CommandScheduler.
 * A Command is a bit of code that can be scheduled to run
 * in sequence or in parallel
 */
public abstract class Command {
    protected boolean isScheduled = false;

    public abstract void cancel();
    public abstract void execute();
    public abstract void initialize();
    public abstract void end();
    public abstract boolean isFinished();

    public SequentialCommandGroup andThen(Command next) {
        return new SequentialCommandGroup(this, next);
    }

    public SequentialCommandGroup andThen(Runnable next) {
        return new SequentialCommandGroup(this, new RunnableCommand(next));
    }

    public boolean isScheduled() {
        return isScheduled;
    }
    void schedule() {
        isScheduled = true;
    }
}
