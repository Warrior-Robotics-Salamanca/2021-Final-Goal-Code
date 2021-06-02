package org.firstinspires.ftc.teamcode.GameOpModes.Tests.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotContainer;

@Disabled
@TeleOp(group="Tuning")
public class TuneRingBlocker extends LinearOpMode {
    RobotContainer robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotContainer(this);

        waitForStart();

        while(opModeIsActive()){
            robot.intake.setRingBlockerUp();
            sleep(1000);
            robot.intake.setRingBlockerDown();
            sleep(1000);
        }
        robot.shutdown();
    }
}
