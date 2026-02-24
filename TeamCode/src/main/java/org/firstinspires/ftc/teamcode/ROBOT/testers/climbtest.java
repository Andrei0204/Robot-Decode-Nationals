package org.firstinspires.ftc.teamcode.ROBOT.testers;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class climbtest extends LinearOpMode {
    Servo test;
    Servo test1;

    public static double pos =0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"CLIMB1");
        test1 =hardwareMap.get(Servo.class,"CLIMB2");
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



