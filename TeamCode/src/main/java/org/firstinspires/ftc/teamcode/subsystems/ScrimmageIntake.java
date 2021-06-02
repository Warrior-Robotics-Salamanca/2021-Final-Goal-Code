package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.WobbleGoalLifterConstants;

@Config
public class ScrimmageIntake {
    public DcMotorEx intakeLift;
    public Servo clampServo;
    private int lifterHomePosition = 0;

    private static final int LIFT_HOME_POSITION = 0;
    private static final int LIFT_UP_POSITION = -23;
    private static final int LIFT_DOWN_POSITION = -545;

    public static double SERVO_MAX = 1;
    public static double SERVO_MIN = 0.55;

    public ScrimmageIntake(DcMotor intakeLift, Servo clampServo) {
        this.intakeLift = (DcMotorEx)intakeLift;
        this.clampServo = clampServo;

        lifterHomePosition = LIFT_HOME_POSITION;
    }

    /**
     * sets lifter up and inside the robot
     */
    public void setLifterUp() {
        setLifterPosition(LIFT_UP_POSITION);
    }

    /**
     * sets lifter down and ready to pick up rings
     */
    public void setLifterDown() {
        setLifterPosition(LIFT_DOWN_POSITION);
    }

    public void setLifterPosition(int position) {
        intakeLift.setTargetPosition(position - lifterHomePosition);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeLift.setPower(.25);
    }

    /**
     * sets the current position of the lifter to be the home position
     */
    public void setHomePosition() {
        setHomePosition(intakeLift.getCurrentPosition());
    }

    /**
     * sets the specified posiiton to be the home position of the launcher
     */
    public void setHomePosition(int homePosition) {
        lifterHomePosition = homePosition;
    }

    public void setGripperOpen() {
        clampServo.setPosition(SERVO_MAX);
    }

    public void setGripperClosed() {
        clampServo.setPosition(SERVO_MIN);
    }
}
