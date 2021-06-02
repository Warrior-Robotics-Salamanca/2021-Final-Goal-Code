package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.Pose2D;

/**
 * The DriveBaseConstants class contains every constant that
 * the drive base of the robot requires to drive properly.
 * They are organized here for convenience of tuning
 */
@Config
public class DriveBaseConstants {
    public static double MAX_DRIVE_VELOCITY = 1.5;     // maximum drive velocity in meters/second
    public static double MAX_DRIVE_ACCELERATION = 10;  // maximum drive acceleration in meters/second/second
    public static final double MAX_ANGULAR_WHEEL_VELOCITY = 10;    // maximum swivel velocity in meters/second
    public static final double MAX_ANGULAR_WHEEL_ACCELERATION = 1; // maximum swivel acceleration in meters/second/second


    public static final double WHEEL_DIAMETER_IN = 4;                        // wheel diameter in inches
    public static final double WHEEL_DIAMETER_M = DistanceUnit.METER.fromInches(WHEEL_DIAMETER_IN); // wheel diameter in meters

    public static final double DRIVE_GEAR_RATIO = 1d/1; // gear ratio between output shaft of drive motor and wheel
    public static final double SWIVEL_GEAR_RATIO = 2d/1; // gear ratio between output shaft of swivel motor and swerve unit
    public static final int    TICKS_PER_MOTOR_REVOLUTION = 480;
    public static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION * DRIVE_GEAR_RATIO;
    public static final double WHEEL_REVOLUTIONS_PER_TICK = 1/TICKS_PER_WHEEL_REVOLUTION;
    public static final double DISTANCE_PER_WHEEL_REVOLUTION = Math.PI * WHEEL_DIAMETER_M;
    public static final double TICKS_PER_SWERVE_UNIT_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION * SWIVEL_GEAR_RATIO;

    public static final int DEAD_WHEEL_TICKS_PER_WHEEL_REVOLUTION = 8192;
    public static final double DEAD_WHEEL_DIAMETER_IN = 2.05; // diameter of wheels used for odometry

    // static position of drive wheels from center of robot in meters
    public static final Position leftWheelPosition = new Position(DistanceUnit.METER, 0, 0.142875, 0, 0);
    public static final Position rightWheelPosition = new Position(DistanceUnit.METER, 0, -0.142875, 0, 0);
    public static final double leftWheelDistanceFromCenter = Math.hypot(leftWheelPosition.y, leftWheelPosition.x);
    public static final double rightWheelDistanceFromCenter = Math.hypot(rightWheelPosition.y, rightWheelPosition.x);

    // dead wheel encoder positions in meters
    public static final Pose2D XDeadWheelPosition = new Pose2D(0.127, -0.0962, 0);
    public static final Pose2D YDeadWheelPosition = new Pose2D(0.0969, 0.0935, Math.PI/2);

    // monte carlo stuff
    public static final Pose2D SENSOR1_OFFSET = new Pose2D(DistanceUnit.mPerInch * (17.5/2 - 1.5), -DistanceUnit.mPerInch * 16.5/2, 3*Math.PI/2);
    public static final Pose2D SENSOR2_OFFSET = new Pose2D(DistanceUnit.mPerInch * 17.5/2, 0, 0);
    public static final Pose2D SENSOR3_OFFSET = new Pose2D(DistanceUnit.mPerInch * (17.5/2 - 1.5), DistanceUnit.mPerInch * 16.5/2, Math.PI/2);
    public static final Pose2D SENSOR4_OFFSET = new Pose2D(0, -.1, 0);

    // angle PID controller coefficients
    public static double HEADING_ALIGN_KP = 1;
    public static double HEADING_ALIGN_KI = 1.6;
    public static double HEADING_ALIGN_KD = 7;

    // drive PID controller coefficients
    public static double DRIVE_ALIGN_KP = 2;
    public static double DRIVE_ALIGN_KI = 0;
    public static double DRIVE_ALIGN_KD = 0;
}
