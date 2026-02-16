package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ROBOT.testers.KalmanFilter;
import org.firstinspires.ftc.teamcode.ROBOT.testers.KalmanFilterParameters;

@Config
public class Distance {

    Rev2mDistanceSensor distanceSensor;
    KalmanFilter distanceFilter;
    public enum detection{
        PURPLE,
        GREEN,
        UNKOWN;
    }

    public void sinit(HardwareMap hardwareMap,String string)
    {
        KalmanFilterParameters distanceParams = new KalmanFilterParameters(0.2,0.4);
         distanceFilter = new KalmanFilter(distanceParams);

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class,string);

    }
    public double getDetection(){
        double rawCM = distanceSensor.getDistance(DistanceUnit.MM);

        distanceFilter.update(0, rawCM);

        double filteredDistance = distanceFilter.getState();
        return filteredDistance;
    }
}
