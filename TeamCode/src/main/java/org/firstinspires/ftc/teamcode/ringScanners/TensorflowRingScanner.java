package org.firstinspires.ftc.teamcode.ringScanners;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;

public class TensorflowRingScanner extends RingScanner {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AQ5OAm3/////AAABmZMhUa0p4EXrjo+rr9tv6dl7flp+yTBf1nCVTnjUABtRaRryqxmKNP8rBCD1MYxYUR0cFoR2kvRq2PzwtnaqTDygVa5ZmS+69lVmGSWoh4Vo23dHbpj/S+M92lHQ5hSyrjRhArOLKFkHDVx9Kjm4cfQt5TLrWQoy590HLjCKKj+zIKX/PAkLLqxLqCznIloeiXrycZ8GmyhkeHZTXVAVOlfPE7IYYCDMT4asR79y5yNEda+xN3DgFuasYHSeEPPZG7zqQHBl2BHhVX2/BY6mLS3RotM7xm3tyW7iPM+2ILF/jZ0FRYGJxTDb0E9ufBzA/uMtSTkWFAPF3+LLzzLEtNXI4kaud9cVUMEqGaX7z8mw";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public TensorflowRingScanner(HardwareMap hardwareMap) {
        initVuforia(hardwareMap);
        initTfod(hardwareMap);
    }

    @Override
    public int detectRingStackSize() {
        int stackSize = 0;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel().equals("Single")) {
                        stackSize = 1;
                    } else if(recognition.getLabel().equals("Quad")) {

                    }
                }
            }
        }
        return stackSize;
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public TFObjectDetector getTfod() {
        return this.tfod;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if(tfod != null) {
            tfod.activate();
        }
    }

    public void shutdown() {
        if(tfod != null) {
            tfod.shutdown();
        }
    }
}
