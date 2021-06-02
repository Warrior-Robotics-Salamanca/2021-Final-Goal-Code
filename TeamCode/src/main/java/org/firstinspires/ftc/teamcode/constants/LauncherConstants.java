package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Pose2D;

/**
 * The LauncherConstants class contains every constant that
 * the launcher of the robot requires to operate properly.
 * They are organized here for convenience of tuning
 */
@Config
public class LauncherConstants {
    public static double MAX_LAUNCH_VELOCITY = 4500; // experimentally determined velocity so as not to go over 16 ft max distance limit (even though we can :-))

    public static final double LAUNCH_MOTOR_TICKS_PER_REVOLUTION = 103.6;
    public static final double LAUNCHER_GEAR_RATIO = 3.0/1.0;
    public static final double LAUNCHER_TICKS_PER_REVOLUTION = LAUNCH_MOTOR_TICKS_PER_REVOLUTION / LAUNCHER_GEAR_RATIO;

    public static double INDEXER_HOME_POSITION = 0.7;
    public static double INDEXER_LAUNCH_POSITION = 0.55;

    public static long INDEXER_MOVEMENT_DELAY = 250; // ms to wait for servo to arrive at target position
    public static long LAUNCHER_SPEED_UP_TIME = 300; // ms to wait for launch motor to get up to speed

    public static double LAUNCH_OFFSET = Math.toRadians(-4.26); // how much extra the heading needs to be turned to in order to launch
    public static double LAUNCH_TOLERANCE = Math.toRadians(1); // how much error to consider acceptable

    public static final double VELOCITY_TOLERANCE = 10; // 10 RPM of tolerance
}
