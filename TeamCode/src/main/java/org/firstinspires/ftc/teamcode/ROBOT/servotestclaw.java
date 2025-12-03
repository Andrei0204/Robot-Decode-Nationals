package org.firstinspires.ftc.teamcode.ROBOT;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class  servotestclaw extends LinearOpMode {
    Servo test;

    public static double pos =0;

    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"BEC");


        test.setPosition(pos);

        waitForStart();
        while (opModeIsActive()) { // singurul while
            test.setPosition(pos);
            telemetry.addData("power",test.getPosition());
            telemetry.update();
        }

    }
}



