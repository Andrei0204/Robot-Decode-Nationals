package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Distance {

    Rev2mDistanceSensor distanceSensor;

    public enum detection{
        PURPLE,
        GREEN,
        UNKOWN;
    }

    public void sinit(HardwareMap hardwareMap,String string)
    {

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class,string);

    }
    public double getDetection(){
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
}
