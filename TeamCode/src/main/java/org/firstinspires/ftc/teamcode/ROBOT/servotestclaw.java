package org.firstinspires.ftc.teamcode.ROBOT;



import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class  servotestclaw extends LinearOpMode {
    Servo test;
    Servo test1;

    public static double pos =0;
    public static double pos2 =0;

    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"tur1");
        test1 =hardwareMap.get(Servo.class,"tur2");

        test.setPosition(0);
        test1.setPosition(0);

        waitForStart();
        while (opModeIsActive()) { // singurul while
            test.setPosition(pos);
            test1.setPosition(pos2);
            telemetry.update();
        }

    }
}



