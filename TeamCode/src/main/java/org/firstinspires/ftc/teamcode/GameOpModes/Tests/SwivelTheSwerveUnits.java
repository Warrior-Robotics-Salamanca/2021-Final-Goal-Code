package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

//@Disabled//
@TeleOp(name="swivel swerve", group="Tuning")
public class SwivelTheSwerveUnits extends LinearOpMode {
    RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        waitForStart();

        robot.driveBase.rightUnit.setSwivelVelocity(2 * Math.PI);
        robot.driveBase.leftUnit.setSwivelVelocity(2 * Math.PI);

        sleep(5000);

        robot.driveBase.leftUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);
        robot.driveBase.rightUnit.setSwivelAngleCircular(0, AngleUnit.RADIANS);

        while(opModeIsActive());
        robot.shutdown();
    }
}
