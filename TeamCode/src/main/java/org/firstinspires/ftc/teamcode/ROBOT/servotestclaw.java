package org.firstinspires.ftc.teamcode.ROBOT;



import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class  servotestclaw extends LinearOpMode {
    ServoImplEx test;
//    Servo test1;

    public static double pos =0;
//    public static double pos2 =0;

    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(ServoImplEx.class,"PIVOT");
//        test1 =hardwareMap.get(Servo.class,"tur2");
//        test.setPwmRange(new PwmControl.PwmRange(500,2500));

//        test1.setPosition(0);

        waitForStart();
        while (opModeIsActive()) { // singurul while
            double max =1;
            double min=0.1;
            double degmin=25;
            double degmax=58;
            double grd =max+(pos-degmin)/(degmax-degmin)*(min-max);
            if(grd<0.08)
                grd=0.08;
            if(grd>1)
                grd=1;
            test.setPosition(grd);
//            test1.setPosition(pos2);
            telemetry.update();
        }

    }
}



