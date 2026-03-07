package org.firstinspires.ftc.teamcode.ROBOT.testers;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.LTur;

@Config
@TeleOp
public class turrettest extends LinearOpMode {
    Servo T1;
    Servo T2;
    DcMotor turret;
    public static final double TICKS435 = 8192;    //test
    public static final double TURTICKS = TICKS435 * 4.5;    //test
    public static double TICKSDEG = TURTICKS / 360;
    public static double Kp = 0.015;           // Proportional gain
    public static double Kd = 0.001;          // Derivative gain
    public static double Ki = 0.001;          // Derivative gain
    public static double targetAngle = 0;     // Target angle in encoder ticks
    public static double target = 0;

    public PIDFController controller = new PIDFController(new PIDFCoefficients(Kp, Ki, Kd, 0));

    @Override
    public void runOpMode() throws InterruptedException {
//        T1 = hardwareMap.get(Servo.class, "T2");
//        T2 = hardwareMap.get(Servo.class, "T1");
//        turret = hardwareMap.get(DcMotor.class, "INTAKE");
//        turret.setDirection(DcMotorSimple.Direction.REVERSE);
//        T2.setDirection(Servo.Direction.REVERSE);
//        T1.setDirection(Servo.Direction.REVERSE);
//        waitForStart();
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTur kaka = new LTur(hardwareMap, org.firstinspires.ftc.teamcode.ROBOT.Modules.LTur.State.AUTO);
        kaka.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kaka.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) { // singurul while\\
//
//            setTargetAngle(target);
//            controller.setP(Kp);
//            controller.setI(Ki);
//            controller.setD(Kd);
//            double currentAngle = turret.getCurrentPosition() / TICKSDEG; // in ticks
//            controller.setTargetPosition(targetAngle);
//            controller.updatePosition(currentAngle);
//            // ----- PD CONTROL -----
//            double power = controller.run();
//            // ----- CLAMP POWER -----
//            power = Math.max(-1.0, Math.min(1.0, power));
//            // ----- SEND TO SERVO -----
//            // Continuous servo expects 0-1 scale where 0.5 = stop
//            double servoCommand = 0.5 + power / 2.0;
//            T1.setPosition(servoCommand);
//            T2.setPosition(servoCommand);
//            // ----- TELEMETRY -----
//            telemetry.addData("Target", targetAngle);
//            telemetry.addData("Current", currentAngle);
//            telemetry.addData("Power", power);
//            telemetry.update();

            kaka.setangle(targetAngle);
            kaka.update();
        }


    }

}




