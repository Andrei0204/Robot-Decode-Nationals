package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class ShooterConstants {
    public static double x = 135; //inches
    public static double y = -138; //inches

    public static Pose GOAL_POS_RED = new Pose(x, y);
    public static Pose GOAL_POS_BLUE = new Pose(140, -10);
    public static double SCORE_HEIGHT = 28; //inches
    public static double SCORE_ANGLE =-0.19987755982988; //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches
}

