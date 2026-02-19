package org.firstinspires.ftc.teamcode.ROBOT.testers;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class  servotestclaw extends LinearOpMode {
    Servo test;
    Servo test1;

    public static double pos =0;
    public static double pos2 =0;

    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"tur");
        test1 =hardwareMap.get(Servo.class,"tur2");
//        test.setPwmRange(new PwmControl.PwmRange(500,2500));

//        test1.setPosition(0);

        waitForStart();
        while (opModeIsActive()) { // singurul while
            test.setPosition(pos);
            test1.setPosition(pos);
            telemetry.update();
        }

    }
}



