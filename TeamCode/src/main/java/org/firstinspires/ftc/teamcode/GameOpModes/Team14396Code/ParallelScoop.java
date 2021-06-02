package org.firstinspires.ftc.teamcode.GameOpModes.Team14396Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ParallelScoop extends LinearOpMode {
    DcMotorEx angle1;
    DcMotorEx angle2;

    int angle1Pos = 0;
    int angle2Pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        angle1 = hardwareMap.get(DcMotorEx.class, "motor1");
        angle2 = hardwareMap.get(DcMotorEx.class, "motor2");

        waitForStart();

        while (opModeIsActive()) {
            angle1.setTargetPosition(angle1Pos);
            angle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            angle1.setPower(1);

            angle2.setTargetPosition(-angle1.getCurrentPosition() + angle2Pos);
            angle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            angle2.setPower(1);

            angle1Pos += gamepad2.right_stick_y * 10;
            angle2Pos += gamepad2.left_stick_y * 10;
        }
    }
}
