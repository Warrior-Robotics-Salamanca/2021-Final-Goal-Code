package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

public class IntakeAndTransfer {
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private Servo ringBlocker;
    private Servo transferServo;

    public IntakeAndTransfer(DcMotor intakeMotor, DcMotor transferMotor, Servo ringBlocker, Servo transferServo) {
        this.intakeMotor = intakeMotor;
        this.transferMotor = transferMotor;
        this.ringBlocker = ringBlocker;
        this.transferServo = transferServo;
    }

    public void setIntakePower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        intakeMotor.setPower(power);
        transferMotor.setPower(power);
        transferServo.setPosition((power+1.0)/2.0);
    }

    public void setRingBlockerDown() {
        ringBlocker.setPosition(IntakeConstants.RING_BLOCKER_DOWN_POSITION);
    }

    public void setRingBlockerUp() {
        ringBlocker.setPosition(IntakeConstants.RING_BLOCKER_UP_POSITION);
    }
}
