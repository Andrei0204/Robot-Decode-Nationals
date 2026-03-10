package org.firstinspires.ftc.teamcode.ROBOT.Teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.ShooterConstants;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Angles;

import java.util.List;
import java.util.Locale;

@Config
@Configurable
@TeleOp
public class TeleopB extends OpMode {
    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    DcMotor BR;
    DcMotor BL;
    DcMotor FL;
    DcMotor FR;
    ElapsedTime timer;
    public static double x = -6, y = -148;
    RobotControl control;
    RoboModules modules;
    public int spin = 0;
    public static double multv = 0.35;
    public static double mult = 0.2;
    public int slow = 0;
    public int cooldown = 750;
    double oldTime = 0;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        odo.setOffsets(x, y, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        modules = new RoboModules(hardwareMap);
        control = new RobotControl(gamepad1, gamepad2, modules, odo);
        timer = new ElapsedTime();
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
        control.red=0;
        control.auto=0;
        ShooterConstants.restartpos();
        modules.sensorstart();
//        c1= hardwareMap.get(NormalizedColorSensor.class,"C1");
//        c2= hardwareMap.get(NormalizedColorSensor.class,"C2");
//        c3= hardwareMap.get(NormalizedColorSensor.class,"C3");
    }

    @Override
    public void init_loop() {
        if (gamepad2.a) {
            odo.resetPosAndIMU();
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 7.7, 7.5, AngleUnit.DEGREES, 90));
        }

    }

    @Override
    public void start() {

//        modules.lTur.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        modules.lTur.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));

        if (gamepad2.a)
            control.setSwm(1);
        if (gamepad2.b)
            control.setSwm(0);
        if (gamepad2.dpadUpWasPressed())
            ShooterConstants.SCORE_HEIGHT_FAR = ShooterConstants.SCORE_HEIGHT_FAR + 0.5;
        if (gamepad2.dpadDownWasPressed())
            ShooterConstants.SCORE_HEIGHT_FAR = ShooterConstants.SCORE_HEIGHT_FAR - 0.5;
        if (gamepad2.dpadLeftWasPressed())
            ShooterConstants.GOAL_POS_BLUE = new Pose(ShooterConstants.GOAL_POS_BLUE.getX() - 1, ShooterConstants.GOAL_POS_BLUE.getY());
        if (gamepad2.dpadRightWasPressed())
            ShooterConstants.GOAL_POS_BLUE = new Pose(ShooterConstants.GOAL_POS_BLUE.getX() + 1, ShooterConstants.GOAL_POS_BLUE.getY());
        if (gamepad2.leftBumperWasPressed())
            ShooterConstants.SCORE_HEIGHT_CLOSE = ShooterConstants.SCORE_HEIGHT_CLOSE - 0.5;
        if (gamepad2.rightBumperWasPressed())
            ShooterConstants.SCORE_HEIGHT_CLOSE = ShooterConstants.SCORE_HEIGHT_CLOSE + 0.5;
        if (gamepad1.share) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 7.7, 7.5, AngleUnit.DEGREES, 90));
        }
        if (gamepad2.y) {
            double h = pos.getHeading(AngleUnit.DEGREES);
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 73, 22, AngleUnit.DEGREES, h));
            ShooterConstants.restartpos();
        }
        if (gamepad2.x) {
            double h = pos.getHeading(AngleUnit.DEGREES);
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 73, 77, AngleUnit.DEGREES, h));
            ShooterConstants.restartpos();
        }
        odo.update();
        if (false) {
            mult = multv;
        } else
            mult = 1;
        double x = -gamepad1.left_stick_x * mult;
        double y = gamepad1.left_stick_y * mult;
        double rx = -gamepad1.right_stick_x * mult;
        if(control.ishanging==1) {
            double headingDiff = Angles.normalize((3*Math.PI)/4 - pos.getHeading(AngleUnit.RADIANS));
            PIDController hpid = new PIDController(1, 0, 0.11);
            rx = hpid.calculate(-headingDiff);
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        FL.setPower((y + x + rx) / denominator);
        FR.setPower((y - x - rx) / denominator);
        BL.setPower((y - x + rx) / denominator);
        BR.setPower((y + x - rx) / denominator);
//
//
        modules.update();
        control.update();

        telemetry.addData("Position", data);
        telemetry.addData("TARGET", ShooterConstants.GOAL_POS_BLUE);

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.addData("vector", control.robotToGoalVector.toString()); //prints the control system refresh rate



        telemetry.update();
    }
}



