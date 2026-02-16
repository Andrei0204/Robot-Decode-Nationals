package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ROBOT.testers.KalmanFilter;
import org.firstinspires.ftc.teamcode.ROBOT.testers.KalmanFilterParameters;

@Config
public class DistanceC {

    RevColorSensorV3 distanceSensor;

    public enum detection {
        PURPLE,
        GREEN,
        UNKOWN;
    }

    KalmanFilter distanceFilter;

    public void sinit(HardwareMap hardwareMap, String string) {
        KalmanFilterParameters distanceParams = new KalmanFilterParameters(0.2, 0.4);
        distanceFilter = new KalmanFilter(distanceParams);

        distanceSensor = hardwareMap.get(RevColorSensorV3.class, string);
    }

    public double getDetection() {
        double rawCM = distanceSensor.getDistance(DistanceUnit.MM);

        distanceFilter.update(0, rawCM);

        double filteredDistance = distanceFilter.getState();
        return filteredDistance;
    }
}
