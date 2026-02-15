package org.firstinspires.ftc.teamcode.ROBOT;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl;

import java.util.Arrays;
import java.util.Locale;

@Config
@Configurable
@TeleOp
public class Teleop extends OpMode {
    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    DcMotor BR;
    DcMotor BL;
    DcMotor FL;
    DcMotor FR;
    ElapsedTime timer;
    public static double x=-6,y=-148;
    RobotControl control;
    RoboModules modules;
    public int spin = 0;
    public static double multv = 0.35;
    public static double mult = 0.2;
    public int slow = 0;
    public int cooldown = 750;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pp");
        odo.setOffsets(x, y, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        modules = new RoboModules(hardwareMap);
        control = new RobotControl(gamepad1, gamepad2, modules,odo);
        timer= new ElapsedTime();
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        c1= hardwareMap.get(NormalizedColorSensor.class,"C1");
//        c2= hardwareMap.get(NormalizedColorSensor.class,"C2");
//        c3= hardwareMap.get(NormalizedColorSensor.class,"C3");
    }
@Override public void start(){
    odo.setPosition(new Pose2D(DistanceUnit.INCH,7.7,7.5,AngleUnit.DEGREES,90));
    modules.lTur.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    modules.lTur.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    modules.sensorstart();
}
    @Override
    public void loop() {
        if(gamepad1.share)
        {
            odo.setPosition(new Pose2D(DistanceUnit.INCH,7.7,7.5, AngleUnit.DEGREES,90));
        }
        odo.update();
        if(gamepad1.left_stick_button)
        {
         mult=multv;
        }
        else
            mult=1;
        double x = -gamepad1.left_stick_x*mult;
        double y = gamepad1.left_stick_y*mult;
        double rx = -gamepad1.right_stick_x*mult;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FL.setPower((y + x + rx) / denominator);
        FR.setPower((y - x - rx) / denominator);
        BL.setPower((y - x + rx) / denominator);
        BR.setPower((y + x - rx) / denominator);



        modules.update();
        control.update();

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);




        telemetry.addData("spinner", modules.launcher.Motorvel());
        telemetry.addData("spinner", modules.launcher.getLaunchtoRpm(control.flywheelSpeed));

        telemetry.addData("spinner", modules.spinner.getRealCurrentPosition());
        telemetry.addData("spinner", modules.spinner.getRealCurrentPosition());
        telemetry.addData("spinner2", modules.spinner.getTarget());
        telemetry.addData("robotToGoalVector", control.robotToGoalVector);
        telemetry.addData("sdas", modules.spinner.spinner.motor.motor.getCurrent(CurrentUnit.AMPS));

        telemetry.update();
    }
}



