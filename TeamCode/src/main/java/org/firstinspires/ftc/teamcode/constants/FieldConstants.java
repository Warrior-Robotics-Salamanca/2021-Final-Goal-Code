package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Pose2D;

/**
 * The FieldConstants class contains constants pertaining
 * to the FTC field and field layout.
 */
@Config
public class FieldConstants {
    public static double WallToWallWidth_M = DistanceUnit.mPerInch * 12 * 12;
    public static final double CenterToWallWidth_M = WallToWallWidth_M/2;
    public static final double CenterToLeftWallWidth_M = CenterToWallWidth_M/3;

    public static final Pose2D LAUNCH_GOAL_CENTER = new Pose2D(1.5*24* DistanceUnit.mPerInch, 3*24*DistanceUnit.mPerInch, 0);

    public static final Pose2D PEG1_CENTER = new Pose2D(2.5*DistanceUnit.mPerInch, 3*24*DistanceUnit.mPerInch, 0);
    public static final Pose2D PEG2_CENTER = new Pose2D(10*DistanceUnit.mPerInch, 3*24*DistanceUnit.mPerInch, 0);
    public static final Pose2D PEG3_CENTER = new Pose2D(17.5*DistanceUnit.mPerInch, 3*24*DistanceUnit.mPerInch, 0);
}
