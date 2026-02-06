package org.firstinspires.ftc.teamcode.ROBOT;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Pivot;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.ShooterConstants;

import java.util.Locale;

@Config
@TeleOp
public class testlaunchall extends OpMode {


    public static double kaka = 0;
    public static double clamp = 47;
    public static double oldTime = 0;
    public static double kaka2 = 0;
    Launcher launcher = null;
    Pivot piv = null;
    LTur tur = null;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu
    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pp");
        tur = new LTur(hardwareMap, LTur.State.AUTO);
        tur.setangle(0);
        tur.update();
        odo.setOffsets(-5, -155, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.INCH));
        telemetry.update();

        resetRuntime();

        odo.setPosition(new Pose2D(DistanceUnit.INCH,7.7,-7.5, AngleUnit.DEGREES,0));
        launcher = new Launcher(hardwareMap, Launcher.State.START);
        piv = new Pivot(hardwareMap, Pivot.State.START);
        launcher.setState(Launcher.State.TEST);
        piv.setState(Pivot.State.AUTO);
        launcher.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start(){
        odo.setPosition(new Pose2D(DistanceUnit.INCH,7.7,-7.5,AngleUnit.DEGREES,0));
        tur.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tur.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double normalizaion(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }
    @Override
    public void loop() {

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;



        Vector robotToGoalVector = new Vector(  new Pose(ShooterConstants.GOAL_POS_RED.getX()-odo.getPosX(DistanceUnit.INCH) ,ShooterConstants.GOAL_POS_RED.getY()-odo.getPosY(DistanceUnit.INCH) ) );
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

//calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(30),
                    Math.toRadians(65));

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));


        piv.setangle(90-Math.toDegrees(hoodAngle));
        launcher.LaunchtoRpm(flywheelSpeed);
        double deltax = +odo.getPosX(DistanceUnit.INCH) - ShooterConstants.GOAL_POS_RED.getX();
        double deltay = -odo.getPosY(DistanceUnit.INCH) + ShooterConstants.GOAL_POS_RED.getY();

        double impartire = deltay / deltax;
        double  alpha = -Math.atan(impartire);

        double anglelocal = odo.getHeading(AngleUnit.RADIANS) - alpha;
        double anglenorm = Math.toDegrees(normalizaion(anglelocal));
        if (anglenorm <= -140)
            anglenorm += 360;

        tur.setangle(kaka2+anglenorm);
        tur.update();
        launcher.setState(Launcher.State.TEST);
        telemetry.addData("fr", frequency);
        telemetry.addData("angle", anglenorm);

        telemetry.addData("velocity", launcher.Motorvel());
        telemetry.addData("kaka", kaka*100);
        telemetry.addData("robotToGoalVector", robotToGoalVector.getMagnitude());
        telemetry.addData("theta", robotToGoalVector.getTheta());
        telemetry.addData("flyheelspeed", flywheelSpeed);
        telemetry.addData("hoodAngle", Math.toDegrees(hoodAngle));
        telemetry.addData("X", ShooterConstants.GOAL_POS_RED.getX()-odo.getPosX(DistanceUnit.INCH) );
        telemetry.addData("Y", ShooterConstants.GOAL_POS_RED.getY()-odo.getPosY(DistanceUnit.INCH) );



        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        odo.update();
        piv.update();
        launcher.update();
        telemetry.update();

    }
}



