package org.firstinspires.ftc.teamcode.GameOpModes.Tests.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.LaunchRings;
import org.firstinspires.ftc.teamcode.commands.RunnableCommand;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.util.Button;

import java.util.concurrent.atomic.AtomicBoolean;

//@Disabled
@TeleOp(group="Tuning")
@Config
public class TuneLaunchVelocity extends LinearOpMode {
    public static double launchVelocity = 1000; // rpm

    public static double VELOC_KP = 60;
    public static double VELOC_KI = 1.4;
    public static double VELOC_KD = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //CommandScheduler.getInstance().enable();

        DcMotor launchMotor = hardwareMap.get(DcMotor.class, "launch_motor");
        Servo ringIndexer = hardwareMap.get(Servo.class, "ring_indexer");
        Launcher launcher = new Launcher(launchMotor, ringIndexer);
        launcher.setIndexerHome();

        Button indexButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.a;
            }
        };
        indexButton.onButtonPressed(new LaunchRings(launcher, 3));

        AtomicBoolean launcherOn = new AtomicBoolean(true);
        Button toggleLauncherButton = new Button() {
            @Override
            public boolean get() {
                return gamepad1.x;
            }
        };
        toggleLauncherButton.onButtonPressed(new RunnableCommand(() -> {
            launcherOn.set(!launcherOn.get());
        }));

        CommandScheduler.getInstance().addButtons(indexButton);

        //launcher.launchMotor.setVelocityPIDFCoefficients(50, 3, 0, 0);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            launcher.launchMotor.setVelocityPIDFCoefficients(VELOC_KP, VELOC_KI, VELOC_KD, 0);

            if(launcherOn.get())
                launcher.setLauncherVelocity(launchVelocity);
            else
                launcher.setLauncherPower(0);

            //telemetry.addData("kp", launcher.launchMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

            telemetry.addData("current_velocity", launcher.getLauncherVelocity());
            telemetry.addData("target_velocity", launchVelocity);
            telemetry.update();
        }
        launcher.setLauncherPower(0);
        //CommandScheduler.getInstance().disable();
    }
}
