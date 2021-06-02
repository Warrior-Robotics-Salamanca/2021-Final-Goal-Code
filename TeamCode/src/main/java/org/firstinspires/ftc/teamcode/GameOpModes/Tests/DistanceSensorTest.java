package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

@Disabled
@TeleOp (group = "Tests")
public class DistanceSensorTest extends LinearOpMode {
    private RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("dist1", robot.distanceSensor1.sensor.getDistance(DistanceUnit.METER));
            telemetry.addData("dist2", robot.distanceSensor2.sensor.getDistance(DistanceUnit.METER));
            telemetry.addData("dist3", robot.distanceSensor3.sensor.getDistance(DistanceUnit.METER));
            telemetry.update();
        }
    }
}
