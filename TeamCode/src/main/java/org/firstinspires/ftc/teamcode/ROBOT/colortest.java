package org.firstinspires.ftc.teamcode.ROBOT;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Color;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Distance;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.DistanceC;

@Config
@TeleOp
public class colortest extends OpMode {

    Color color= new Color();
    Color color2= new Color();
    Color color3= new Color();
    Distance distance= new Distance();
    Distance distance2= new Distance();
    Distance distance3= new Distance();

    DistanceC distancec= new DistanceC();
    DistanceC distance2c= new DistanceC();
    DistanceC distance3c= new DistanceC();




    @Override
    public void init() {
        color.sinit(hardwareMap,"C1");
        color2.sinit(hardwareMap,"C2");
        color3.sinit(hardwareMap,"C3");
        distancec.sinit(hardwareMap,"C1");
        distance2c.sinit(hardwareMap,"C2");
        distance3c.sinit(hardwareMap,"C3");
        distance.sinit(hardwareMap,"D1");
        distance2.sinit(hardwareMap,"D2");
        distance3.sinit(hardwareMap,"D3");
    }

    @Override
    public void loop() {

        telemetry.addData("detection",color.getDetection(telemetry));
        telemetry.addData("detection",color2.getDetection(telemetry));
        telemetry.addData("detection",color3.getDetection(telemetry));
        telemetry.addData("d1",distance.getDetection());
        telemetry.addData("d1",distancec.getDetection());
        telemetry.addData("d2",distance2.getDetection());
        telemetry.addData("d2",distance2c.getDetection());
        telemetry.addData("d3",distance3.getDetection());
        telemetry.addData("d3",distance3c.getDetection());
        telemetry.update();

    }
}



