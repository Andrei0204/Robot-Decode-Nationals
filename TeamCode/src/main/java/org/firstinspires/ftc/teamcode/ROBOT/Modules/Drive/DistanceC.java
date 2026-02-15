package org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistanceC {

    RevColorSensorV3 distanceSensor;

    public enum detection{
        PURPLE,
        GREEN,
        UNKOWN;
    }

    public void sinit(HardwareMap hardwareMap,String string)
    {

        distanceSensor = hardwareMap.get(RevColorSensorV3.class,string);
        distanceSensor.red();
    }

    public double getDetection(){
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
}
