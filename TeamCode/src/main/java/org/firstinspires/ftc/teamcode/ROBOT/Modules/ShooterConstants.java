package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable

@Config
public class ShooterConstants {

    public static Pose GOAL_POS_RED = new Pose(126, 133);
    public static Pose GOAL_POS_RED_AUTO = new Pose(132, 132);
    public static Pose GOAL_POS_BLUE = new Pose(24, 140);
    //    public static Pose GOAL_POS_BLUE =new Pose(3 ,132);
    public static Pose GOAL_POS_BLUE_AUTO = new Pose(17, 135);
    public static double SCORE_HEIGHT_FAR = 34.5; //inches
    public static double SCORE_HEIGHT_CLOSE = 32.5; //inches
    public static double SCORE_ANGLE_FAR = -0.35; //radians
    public static double SCORE_ANGLE_CLOSE = -0.34;//21 //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    public static void restartpos() {
        GOAL_POS_BLUE = new Pose(12, 133.5
        );
     SCORE_ANGLE_CLOSE = -0.3;//21 //radians
        SCORE_ANGLE_FAR = -0.3  ; //radians
        GOAL_POS_RED = new Pose(131  , 132);
        SCORE_HEIGHT_FAR = 34; //inches
        SCORE_HEIGHT_CLOSE = 33
        ; //inches
        RobotControl.counterf=240;
        RobotControl.counterc=40;


    }
}

