package org.firstinspires.ftc.teamcode.ROBOT;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testtur extends OpMode {

    public static double kaka = 0;
    public static double kaka2 = 0;
    LTur tur = null;

    @Override
    public void init() {
        tur = new LTur(hardwareMap, LTur.State.START);
        tur.setangle(0);
        tur.setState(LTur.State.AUTO);
        tur.update();
    }

    @Override
    public void start(){
        tur.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tur.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {

        tur.setangle(kaka);
        telemetry.addData("", tur.getCurrentPositionenc());

        tur.update();
        telemetry.update();

    }
}



