package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Button;

import java.util.ArrayList;
import java.util.Collection;

/**
 * The CommandScheduler class contains methods for
 * running commands in the main thread or in a
 * separate thread. It also handles the scheduling
 * of commands tied to button events.
 */
public class CommandScheduler {
    private static final CommandScheduler mainScheduler = new CommandScheduler();

    private final ArrayList<Command> allCommands = new ArrayList<Command>();
    private final ArrayList<Command> commandsToRemove = new ArrayList<Command>();

    private final ArrayList<Button> allButtons = new ArrayList<Button>();

    private Runnable asyncScheduleRunnable = new Runnable() {
        @Override
        public void run() {
            while(isEnabled)
                mainScheduler.run();
        }
    };

    private boolean isEnabled = false;

    public static CommandScheduler getInstance() {
        return mainScheduler;
    }

    public void run() {
        commandsToRemove.clear();
        for(Button button : allButtons) {
            if(button != null)
                button.update();
        }

        allCommands.parallelStream().forEach(command -> {
            if(command != null) {
                if (!command.isScheduled()) {
                    command.schedule();
                    command.initialize();
                }
                command.execute();
                if (command.isFinished()) {
                    command.end();
                    commandsToRemove.add(command);
                }
            }
        });
        allCommands.removeAll(commandsToRemove);
    }

    public void registerSubsystem(Subsystem... subsystems) {

    }

    public void schedule(Command... commands) {
        for(Command c : commands) {
            allCommands.add(c);
        }
    }

    public void addButtons(Button... buttons) {
        for(Button b : buttons) {
            allButtons.add(b);
        }
    }

    public void addButton(Button button) {
        allButtons.add(button);
    }

    public void enable() {
        isEnabled = true;
        new Thread(asyncScheduleRunnable).start();
    }

    public void disable() {
        isEnabled = false;
        cancelAll();
    }

    public void cancelAll() {
        allCommands.clear();
    }
}
