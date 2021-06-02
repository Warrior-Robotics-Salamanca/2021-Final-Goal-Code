package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.WobbleGoalLifterConstants;

public class WobbleGoalLifter {
    private Servo claw;
    private DcMotorEx lifter;
    private int lifterHomePosition = 0;

    public WobbleGoalLifter(DcMotorEx lifter, Servo claw) {
        this.lifter = lifter;
        this.claw = claw;

        setHomePosition(WobbleGoalLifterConstants.ARM_START_POSITION);
    }

    /**
     * sets lifter up and inside the robot
     */
    public void setLifterUp() {
        setLifterPosition(WobbleGoalLifterConstants.ARM_UP_POSITION);
    }

    /**
     * sets lifter down and ready to pick up wobble goals
     */
    public void setLifterDown() {
        setLifterPosition(WobbleGoalLifterConstants.ARM_DOWN_POSITION);
    }

    public void setLifterPosition(int position) {
        lifter.setTargetPosition(position - lifterHomePosition);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(.4);
    }

    public int getLifterPosition() {
        return lifter.getCurrentPosition();
    }

    /**
     * sets the current position of the lifter to be the home position
     */
    public void setHomePosition() {
        setHomePosition(lifter.getCurrentPosition());
    }

    /**
     * sets the specified posiiton to be the home position of the launcher
     */
    public void setHomePosition(int homePosition) {
        lifterHomePosition = homePosition;
    }

    public void setClawOpen() {
        claw.setPosition(.5);
    }

    public void setClawClosed() {
        claw.setPosition(0);
    }

    public boolean isBusy() {
        return lifter.isBusy();
    }
}
