package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Position {
    public static Pose2d pose;
    public Position(Pose2d pose){
        this.pose= pose;
    }
    public Position(){
    }

    public static Pose2d getPose() {
        return pose;
    }

    public static void setPose(Pose2d pose) {
        Position.pose = pose;
    }
}
