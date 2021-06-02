package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

@Disabled
@TeleOp
public class TestRobotSpin extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotContainer robot = new RobotContainer(this);

        waitForStart();

        robot.driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
        robot.driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
        robot.driveBase.leftUnit.setDrivePower(-1);
        robot.driveBase.rightUnit.setDrivePower(1);

        while(opModeIsActive());
    }
}
