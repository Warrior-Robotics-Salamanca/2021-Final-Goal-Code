package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.LauncherConstants;

public class Launcher {
    public DcMotorEx launchMotor;
    private Servo indexer;

    private double setPoint;

    public enum LauncherTarget {
        POWERSHOT,
        HIGH_GOAL
    }

    public Launcher(DcMotor launchMotor, Servo indexer) {
        this.launchMotor = (DcMotorEx)launchMotor;
        this.indexer = indexer;
        setPoint = 0;
    }

    public boolean isUpToSpeed() {
        return Math.abs(setPoint - getLauncherVelocity()) < LauncherConstants.VELOCITY_TOLERANCE;
    }

    public void setIndexerHome() {
        indexer.setPosition(LauncherConstants.INDEXER_HOME_POSITION);
    }

    public void setIndexerLaunch() {
        indexer.setPosition(LauncherConstants.INDEXER_LAUNCH_POSITION);
    }

    public void setLauncherPower(double power) {
        launchMotor.setPower(-Math.abs(power));
    }

    public void setLauncherVelocity(double rpm) {
        setPoint = -Math.abs(Math.min(rpm, LauncherConstants.MAX_LAUNCH_VELOCITY));
        launchMotor.setVelocity(setPoint*LauncherConstants.LAUNCHER_TICKS_PER_REVOLUTION/(60));
    }

    public double getLauncherVelocity() {
        return -launchMotor.getVelocity()*60/LauncherConstants.LAUNCHER_TICKS_PER_REVOLUTION;
    }

    private final double voltToPowerSlope = (.92-.8) / (11.96-13.62);
    private final double voltToPowerOffset = -11.96 * voltToPowerSlope + .92;
    public void setPowerFromVoltage(double nowVoltage) {
        // 11.96 V -> .92
        // 13.62 -> .8
        launchMotor.setPower(-(voltToPowerSlope * nowVoltage + voltToPowerOffset));
    }

    public void setVelocityFromDistance(double distance, LauncherTarget target) {
        if(target.equals(LauncherTarget.HIGH_GOAL))
            //setLauncherVelocity(0.00022772439 * Math.pow(distance,4)-0.101918177659 * Math.pow(distance, 3)+17.037103 * Math.pow(distance, 2)-1256.668 * distance+37734.0246 - (target.equals(LauncherTarget.POWERSHOT) ? 200 : 0));
            setLauncherVelocity(18995.698465007437-578.69036323876696*distance+7.8137955119559690*Math.pow(distance, 2)-.046389543228328618*Math.pow(distance, 3)+.00010282091156267*Math.pow(distance, 4));
        else if(target.equals(LauncherTarget.POWERSHOT)) {
            setLauncherVelocity(-8031.0775215272861+518.15780856868003*distance-8.9458166052162262*Math.pow(distance, 2)+.066130227368704206*Math.pow(distance, 3)-.00017616372816412686*Math.pow(distance, 4));
        }
    }
}
