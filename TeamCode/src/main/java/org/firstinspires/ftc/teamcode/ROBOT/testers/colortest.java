package org.firstinspires.ftc.teamcode.ROBOT.testers;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Color;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Distance;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.DistanceC;

@Config
@TeleOp
public class colortest extends OpMode {

//    Color color= new Color();
//    Color color2= new Color();
//    Color color3= new Color();
//    Distance distance= new Distance();
//
//    Distance distance3= new Distance();
//
//    DistanceC distancec= new DistanceC();
//
//    DistanceC distance3c= new DistanceC();

    DistanceC distance2c = new DistanceC();

    Distance distance2 = new Distance();

    @Override
    public void init() {
//        color.sinit(hardwareMap,"C1");
//        color2.sinit(hardwareMap,"C2");
//        color3.sinit(hardwareMap,"C3");
//        distancec.sinit(hardwareMap,"C1");
        distance2c.sinit(hardwareMap, "C2");
//        distance3c.sinit(hardwareMap,"C3");
//        distance.sinit(hardwareMap,"D1");
        distance2.sinit(hardwareMap, "D2C");
//        distance3.sinit(hardwareMap,"D3");
    }

    @Override
    public void loop() {

        telemetry.addData("d2", distance2.getDetection());
        telemetry.addData("d2", distance2c.getDetection());

        telemetry.update();

    }
}



