package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * A SequentialCommandGroup can be created with
 * a list or array of commands, or by calling the
 * .andThen method of a Command. Commands that are
 * part of a SequentialCommandGroup are executed
 * in the sequence they were passed.
 */
public class SequentialCommandGroup extends Command {
    private ArrayList<Command> allCommands;

    private int currentCommand = 0;

    private boolean isCancelled = false;
    private boolean allCommandsFinished = false;

    public SequentialCommandGroup(Command... commands) {
        allCommands = new ArrayList<Command>(Arrays.asList(commands));
    }

    @Override
    public void cancel() {
        isCancelled = true;
    }

    @Override
    public void execute() {
        Command command = allCommands.get(currentCommand);

        if(!allCommandsFinished) {
            if (!command.isScheduled()) {
                command.schedule();
                command.initialize();
            }
            command.execute();
            if (command.isFinished()) {
                command.end();
                if (currentCommand == allCommands.size() - 1)
                    allCommandsFinished = true;
                else
                    currentCommand++;
            }
        }
    }

    @Override
    public void initialize() {
        Command command = allCommands.get(currentCommand);
        if(!command.isScheduled()) {
            command.schedule();
            command.initialize();
        }
    }

    @Override
    public void end() {
        isScheduled = false;
    }

    @Override
    public boolean isFinished() {
        return isCancelled || allCommandsFinished;
    }

    public void addCommands(Command... commands) {
        for(Command c : commands) {
            allCommands.add(c);
        }
    }
}
