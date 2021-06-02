package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;

public abstract class Button {
    private boolean previousState = false;
    private boolean previousState_internal = false;

    private Command buttonPressedCommand = null;
    private Command buttonReleasedCommand = null;
    private Command buttonHeldCommand = null;

    public Button() {
        previousState = get();
        previousState_internal = previousState;
    }

    /**
     * method that the user should override
     * @return the current state of the button at the time get() is called
     */
    public abstract boolean get();

    public boolean buttonPressed() {
        boolean state = get();
        boolean pressed = previousState != state && !previousState;
        previousState = state;
        return pressed;
    }

    public boolean buttonReleased() {
        boolean state = get();
        boolean pressed = previousState != state && previousState;
        previousState = state;
        return pressed;
    }

    public void onButtonPressed(Command command) {
        buttonPressedCommand = command;
    }

    public void onButtonReleased(Command command) {
        buttonReleasedCommand = command;
    }

    public void whileButtonPressed(Command command) {
        buttonHeldCommand = command;
    }

    public void update() {
        boolean state = get();

        boolean pressed = previousState_internal != state && !previousState_internal;
        boolean released = previousState_internal != state && previousState_internal;

        previousState_internal = state;

        if(pressed && buttonPressedCommand != null && !buttonPressedCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(buttonPressedCommand);
        }
        if(released && buttonReleasedCommand != null && !buttonReleasedCommand.isScheduled()) {
            CommandScheduler.getInstance().schedule(buttonReleasedCommand);
        }
        if(state && buttonHeldCommand != null && !buttonHeldCommand.isScheduled()) {
            buttonHeldCommand.execute();
        }
    }
}
