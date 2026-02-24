package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable

@Config
public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(140, 138);
    public static Pose GOAL_POS_RED_AUTO = new Pose(138, 132);
    public static Pose GOAL_POS_BLUE = new Pose(24, 140);
    //    public static Pose GOAL_POS_BLUE =new Pose(3 ,132);
    public static Pose GOAL_POS_BLUE_AUTO = new Pose(16, 130);
    public static double SCORE_HEIGHT_FAR = 36; //inches
    public static double SCORE_HEIGHT_CLOSE = 33.5; //inches
    public static double SCORE_ANGLE_FAR = -0.17; //radians
    public static double SCORE_ANGLE_CLOSE = -0.21;//21 //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    public static void restartpos() {
        GOAL_POS_BLUE = new Pose(18, 133.5
        );
        SCORE_ANGLE_FAR = -0.19; //radians
        GOAL_POS_RED = new Pose(139 , 130);
        SCORE_HEIGHT_FAR = 35; //inches
        SCORE_HEIGHT_CLOSE = 34.5; //inches


    }
}

