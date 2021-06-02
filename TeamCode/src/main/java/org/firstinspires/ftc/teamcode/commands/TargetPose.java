package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDriveBase;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Pose2D;

/**
 * TargetPose is a class that should be used to target
 * the robot towards a given pose on the field using
 * a PID loop. This class assumes to have full control
 * over the driveBase during this time.
 */
public class TargetPose extends Command {
    private SwerveDriveBase driveBase;
    private Localizer localizer;

    private Pose2D target;

    private boolean isEnded = false;

    private long timeout_ms = 0;
    private ElapsedTime timeoutTimer = new ElapsedTime();

    private static final double MAX_TURN_SPEED = Math.PI; // rad/sec

    private double additionalOffset = 0;

    private PIDController headingController = new PIDController(DriveBaseConstants.HEADING_ALIGN_KP,
            DriveBaseConstants.HEADING_ALIGN_KI,
            DriveBaseConstants.HEADING_ALIGN_KD);

    private Telemetry.Item display1, display2;

    public TargetPose(SwerveDriveBase driveBase, Localizer localizer, Pose2D targetPose, Telemetry.Item display1, Telemetry.Item display2) {
        this.driveBase = driveBase;
        this.localizer = localizer;
        this.target = targetPose;
        this.display1 = display1;
        this.display2 = display2;
        this.timeout_ms = 0;
    }

    public TargetPose(SwerveDriveBase driveBase, Localizer localizer, Pose2D targetPose) {
        this.driveBase = driveBase;
        this.localizer = localizer;
        this.target = targetPose;
        this.display1 = null;
        this.display2 = null;
        this.timeout_ms = 0;
    }

    public TargetPose(SwerveDriveBase driveBase, Localizer localizer, Pose2D targetPose, long timeout_ms) {
        this.driveBase = driveBase;
        this.localizer = localizer;
        this.target = targetPose;
        this.display1 = null;
        this.display2 = null;
        this.timeout_ms = timeout_ms;
    }

    public TargetPose(SwerveDriveBase driveBase, Localizer localizer, Pose2D targetPose, double additionalOffset, long timeout_ms) {
        this.driveBase = driveBase;
        this.localizer = localizer;
        this.target = targetPose;
        this.display1 = null;
        this.display2 = null;
        this.timeout_ms = timeout_ms;
        this.additionalOffset = additionalOffset;
    }

    @Override
    public void cancel() {
        isEnded = true;
    }

    @Override
    public void execute() {
        Pose2D robotPose = localizer.getRobotPose();
        double targetHeading = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX())
                + LauncherConstants.LAUNCH_OFFSET + additionalOffset;

        while(robotPose.getTheta() - targetHeading >= Math.PI) targetHeading += 2 * Math.PI;
        while(robotPose.getTheta() - targetHeading < -Math.PI) targetHeading -= 2 * Math.PI;

        AngularVelocity correction = new AngularVelocity();
        correction.zRotationRate = (float)headingController.getOutput(robotPose.getTheta(), targetHeading);

        if(display1 != null && display2 != null) {
            display1.setCaption("Correction");
            display1.setValue(correction.zRotationRate);

            display1.setCaption("targetHeading");
            display1.setValue(targetHeading);
        }

        if(timeout_ms != 0 && timeoutTimer.milliseconds() >= timeout_ms) {
            driveBase.setVelocityToZero();
            this.end();
        } else {
            Velocity vel = new Velocity();
            vel.xVeloc = 0; vel.yVeloc = 0;
            driveBase.setPowerTankMode(vel, correction);
        }
    }

    @Override
    public void initialize() {
        isEnded = false;
        isScheduled = true;
        headingController.setOutputLimits(-MAX_TURN_SPEED, MAX_TURN_SPEED);

        headingController.setPID(DriveBaseConstants.HEADING_ALIGN_KP,
                DriveBaseConstants.HEADING_ALIGN_KP,
                DriveBaseConstants.HEADING_ALIGN_KP);

        headingController.setMaxIOutput(0.15); // determined experimentally

        Velocity vel = new Velocity();
        vel.xVeloc = 0;
        AngularVelocity ang = new AngularVelocity();
        ang.zRotationRate = 0;
        driveBase.setVelocityTankMode(vel, ang);

        timeoutTimer.reset();
    }

    @Override
    public void end() {
        isEnded = true;
        isScheduled = false;
    }

    @Override
    public boolean isFinished() {
        return false || isEnded;
    }
}
