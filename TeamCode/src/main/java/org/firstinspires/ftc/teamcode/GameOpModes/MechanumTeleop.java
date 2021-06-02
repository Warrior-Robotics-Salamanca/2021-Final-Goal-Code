package org.firstinspires.ftc.teamcode.GameOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.MechanumDriveBase;

@Disabled
@TeleOp(name="mechanum Teleop", group="teamcode")

public class MechanumTeleop extends LinearOpMode {
    private DcMotor frontRight, frontLeft, backRight, backLeft;

    @Override
    public void runOpMode() {

        // drive base motors
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();

        while (opModeIsActive()) {
            // Convert joysticks to desired motion.
            MechanumDriveBase.Wheels wheels = MechanumDriveBase.joystickToWheels(
                          gamepad1.left_stick_x, gamepad1.left_stick_y,
                          gamepad1.right_stick_y);
            wheels.scaleAndMaintainRatio();
            frontLeft.setPower(-wheels.frontLeft);
            frontRight.setPower(-wheels.frontRight);
            backLeft.setPower(-wheels.backLeft);
            backRight.setPower(-wheels.backRight);
        }
    }
}
