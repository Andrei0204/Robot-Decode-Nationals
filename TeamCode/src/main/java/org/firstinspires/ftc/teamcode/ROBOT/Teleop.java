package org.firstinspires.ftc.teamcode.ROBOT;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class Teleop extends OpMode {

    DcMotor BR;
    DcMotor BL;
    DcMotor FL;
    DcMotor FR;

    ElapsedTime timer;
    DcMotor Intake;
    DcMotor L1;
    DcMotor L2;
    Spinner spinner;
    public int spin= 0;

    @Override
    public void init() {
        spinner= new Spinner(hardwareMap, Spinner.State.START);

        timer.reset();
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        Intake = hardwareMap.get(DcMotor.class, "INTAKE");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        L2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BR.setDirection(DcMotorSimple.Direction.REVERSE);
//        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void loop() {
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FL.setPower((y + x + rx) / denominator);
        FR.setPower((y - x - rx) / denominator);
        BL.setPower((y - x + rx) / denominator);
        BR.setPower((y + x - rx) / denominator);

        if(gamepad1.aWasPressed())
        {
            L1.setPower(0);
            L2.setPower(0);
        }



        if(gamepad1.bWasPressed())
        {
            if(spin<0)
                spinner.setAuto((spinner.getRealCurrentPosition()+120));
            else
                spinner.setAuto((spinner.getRealCurrentPosition()+110));
            spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            spinner.setState(Spinner.State.TEST);
        }
        if(gamepad1.xWasPressed())
        {
            L1.setPower(-1);
            L2.setPower(-1);
        }
        if(gamepad1.options) {

            spinner.spinner.resetEncoder();
        }

        if(gamepad1.dpad_down) {
            spin=0;
            spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            spinner.setState(Spinner.State.START);
        }
        if(gamepad1.right_trigger>0.1) {
            spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            spinner.setManualPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger>0.1) {
            spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            spinner.setManualPower(-gamepad1.left_trigger);
        }
        else {
            spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            spinner.setManualPower(0);

        }
        if(gamepad2.leftBumperWasPressed()) {
            if (spin <= -2)
                spin = 2;
            else
                spin--;

            spinner.counter(spin);
        }else if(gamepad2.rightBumperWasPressed())
        {
            if (spin >= 2)
                spin = -2;
            else
                spin++;
            spinner.counter(spin);
        }


        spinner.update();
        telemetry.addData("spinner",spinner.getRealCurrentPosition());
        telemetry.addData("spinner2",spinner.getTarget());
        telemetry.addData("spin",spin);
        telemetry.update();
    }
}



