package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.subsystems.DeadWheelEncoderUnit;

@Disabled
@TeleOp(name="Tick Test", group="Tests")
public class TickTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DeadWheelEncoderUnit unit1 = new DeadWheelEncoderUnit(motor1, DriveBaseConstants.XDeadWheelPosition, 2 * DistanceUnit.mPerInch);
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "intake_motor");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DeadWheelEncoderUnit unit2 = new DeadWheelEncoderUnit(motor2, DriveBaseConstants.YDeadWheelPosition, 2 * DistanceUnit.mPerInch);

        double unit1ApproxPos = 0;

        waitForStart();

        long previousTime = System.nanoTime();
        while(opModeIsActive()) {
            long timeLong = System.nanoTime();
            double dt = (timeLong - previousTime) * 1e-9;
            previousTime = timeLong;

            double vel1 = unit1.getVelocity();
            unit1ApproxPos += vel1*dt;

            telemetry.addData("encoder1 ticks", unit1.getPosition());
            telemetry.addData("encoder1 ticks approx", unit1ApproxPos);
            telemetry.addData("encoder1 veloc", vel1);
            telemetry.addData("encoder2 ticks", unit2.getPosition());
            telemetry.addData("encoder2 veloc", unit2.getVelocity());
            telemetry.update();
        }
    }
}
