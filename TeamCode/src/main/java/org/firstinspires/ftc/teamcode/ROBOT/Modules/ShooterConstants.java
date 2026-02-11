package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable

@Config
public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(135 , 141);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT_FAR = 36; //inches
    public static double SCORE_HEIGHT_CLOSE = 37; //inches
    public static double SCORE_ANGLE_FAR =-0.229982988; //radians
    public static double SCORE_ANGLE_CLOSE =-0.30982988; //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches
}

