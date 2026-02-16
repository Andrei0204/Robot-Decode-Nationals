package org.firstinspires.ftc.teamcode.ROBOT.testers;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Pivot;

@Config
@TeleOp
public class testlaunch extends OpMode {

    public static double kaka = 0;
    public static double kaka2 = 0;
    Launcher tur = null;
    Pivot piv = null;

    @Override
    public void init() {
        tur = new Launcher(hardwareMap, Launcher.State.START);
        piv = new Pivot(hardwareMap, Pivot.State.START);
        tur.setState(Launcher.State.TEST);
        piv.setState(Pivot.State.AUTO);
        tur.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start(){

    }
    @Override
    public void loop() {

        piv.setangle(kaka2);
        tur.setAuto(kaka);
        tur.setState(Launcher.State.TEST);
        telemetry.addData("velocity", tur.Motorvel());
        telemetry.addData("kaka", kaka*100);

        piv.update();
        tur.update();
        telemetry.update();

    }
}



