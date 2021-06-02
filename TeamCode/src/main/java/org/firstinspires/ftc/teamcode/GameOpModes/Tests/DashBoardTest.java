package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Disabled
@Autonomous
public class DashBoardTest extends LinearOpMode {
    public static double AMPLITUDE = 10;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.5;

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AQ5OAm3/////AAABmZMhUa0p4EXrjo+rr9tv6dl7flp+yTBf1nCVTnjUABtRaRryqxmKNP8rBCD1MYxYUR0cFoR2kvRq2PzwtnaqTDygVa5ZmS+69lVmGSWoh4Vo23dHbpj/S+M92lHQ5hSyrjRhArOLKFkHDVx9Kjm4cfQt5TLrWQoy590HLjCKKj+zIKX/PAkLLqxLqCznIloeiXrycZ8GmyhkeHZTXVAVOlfPE7IYYCDMT4asR79y5yNEda+xN3DgFuasYHSeEPPZG7zqQHBl2BHhVX2/BY6mLS3RotM7xm3tyW7iPM+2ILF/jZ0FRYGJxTDb0E9ufBzA/uMtSTkWFAPF3+LLzzLEtNXI4kaud9cVUMEqGaX7z8mw";

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); 
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("x", AMPLITUDE * Math.sin(
                    2 * Math.PI * FREQUENCY * getRuntime() + Math.toRadians(PHASE)
            ));
            telemetry.update();
        }
    }
}