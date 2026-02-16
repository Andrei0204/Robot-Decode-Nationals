package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
@Configurable

@Config
public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(135 , 132);
    public static Pose GOAL_POS_RED_AUTO = new Pose(132 , 132);
    public static Pose GOAL_POS_BLUE =new Pose(3 ,132);
    public static Pose GOAL_POS_BLUE_AUTO =new Pose(3 ,130);
    public static double SCORE_HEIGHT_FAR = 34; //inches
    public static double SCORE_HEIGHT_CLOSE = 33.5; //inches
    public static double SCORE_ANGLE_FAR =-0.185982988; //radians
    public static double SCORE_ANGLE_CLOSE =-0.20;//21 //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches
}

