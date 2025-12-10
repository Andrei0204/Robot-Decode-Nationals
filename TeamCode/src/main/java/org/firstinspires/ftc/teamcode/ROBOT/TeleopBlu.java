package org.firstinspires.ftc.teamcode.ROBOT;

import static org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret.TICKSDEG;
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl.distance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Point;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Vector;

import java.util.List;
import java.util.Locale;


@Config
@TeleOp
public class TeleopBlu extends OpMode {
    DcMotor BL = null, BR = null, FR = null, FL = null;

    RoboModules modules;
    RobotControl control;
    double oldTime = 0;
    public double alpha = 0;
    double deltax = 0;
    double deltay = 0;

    public static double distancemodifier = 1;
    public static double distancemodifierfar = 2;
    public static double deltahhypot = 0;
    public static double kppeleme = 0.34 ;
    Point redGoal = new Point(330, -286);
    Point blueGoal = new Point(333, -6);
    boolean isred = false;
    GoBildaPinpointDriver odo = null;
    Boolean odobased = false;
    Boolean llbased = true;
    ElapsedTime movement;
    Limelight3A limelight;
    LLResult result = null;
    Servo bec;
    double color = 0;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(54, -164.5, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        movement = new ElapsedTime();
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        bec = hardwareMap.get(Servo.class, "BEC");

        modules = new RoboModules(hardwareMap);
        control = new RobotControl(gamepad1, gamepad2, modules);


        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO modifca de aici daca nu se misca bine robotu

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        modules.launcher.turret.setState(Turret.State.TEST);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);
        limelight.start();
    }


    @Override
    public void init_loop() {
        if (gamepad2.a) {
            odo.setPosition(new Pose2D(DistanceUnit.CM, 21, -124, AngleUnit.RADIANS, 0));
            modules.launcher.turret.turret.resetEncoder();
        }
    }

    @Override
    public void start() {

    }


    public double normalizaion(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }

    public void turret() {
        odo.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        if (isred) {
            deltax = +pos.getX(DistanceUnit.CM) - redGoal.x;
            deltay = -pos.getY(DistanceUnit.CM) + redGoal.y;
        } else {
            deltax = +pos.getX(DistanceUnit.CM) - blueGoal.x;
            deltay = -pos.getY(DistanceUnit.CM) + blueGoal.y;
        }

        deltahhypot = Math.hypot(deltay, deltax);
        double impartire = deltay / deltax;
        alpha = -Math.atan(impartire);

        double anglelocal = pos.getHeading(AngleUnit.RADIANS) - alpha;
        double anglenorm = -Math.toDegrees(normalizaion(anglelocal));
        if (anglenorm <= -140)
            anglenorm += 360;


        if (odobased)
            modules.launcher.turret.setAuto((anglenorm) * TICKSDEG);
        if (!odobased && !llbased)
            modules.launcher.turret.setAuto(0 * TICKSDEG);

        if (llbased) {
            double saw = 0;
            result = limelight.getLatestResult();
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> detectorResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult dr : detectorResults) {
                    if (dr.getFiducialId() == 20) {
                        saw=1;
                        if (Math.abs(dr.getTargetXDegrees()) > 8) {
                            color = 0.277;
                        } else if (Math.abs(dr.getTargetXDegrees()) > 4) {
                            color = 0.33;

                        } else {
                            color = 0.5;
                        }
                        telemetry.addData("space?", dr.getRobotPoseTargetSpace());
                        Vector distances= new Vector(dr.getRobotPoseTargetSpace().getPosition().x,dr.getRobotPoseTargetSpace().getPosition().z);

                        distance = distances.getMagnitude()*100;
                        if (movement.milliseconds() > 0 && Math.abs(modules.launcher.turret.getCurrentPosition() - dr.getTargetXDegrees() * TICKSDEG) / TICKSDEG > 1
                        ) {
                            double heading_error = dr.getTargetXDegrees();
                            double goodpower = heading_error * kppeleme;
                            modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - goodpower * TICKSDEG);
                            movement.reset();
                        }
                    }


                }

            }
            if (movement.milliseconds() > 400 && saw==0) {
                modules.launcher.turret.setAuto((anglenorm) * TICKSDEG);
            }

        }

    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            odo.setPosition(new Pose2D(DistanceUnit.CM, 169, -143, AngleUnit.RADIANS, 0));

        }
        color = 0.555;
        turret();
        control.update();
        double x = -gamepad1.left_stick_x; // Remember, Y stick is reversed!
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FL.setPower((y + x + rx) / denominator);
        FR.setPower((y - x - rx) / denominator);
        BL.setPower((y - x + rx) / denominator);
        BR.setPower((y + x - rx) / denominator);


        bec.setPosition(color);
        if (gamepad2.dpad_down) {
            odobased = false;
            llbased = false;
        }
        if (gamepad2.dpad_right) {
            odobased = false;
            llbased = true;
        }
        if (gamepad2.dpad_left) {
            odobased = true;
            llbased = false;
        }


        telemetry.addData("distance", distance);
        telemetry.addData("push", modules.launcher.launcher.Motorvel());
        telemetry.addData("AMPS", modules.launcher.launcher.getAmps());
        telemetry.addData("motor1", modules.launcher.launcher.launcher.getCorrectedVelocity());
        telemetry.addData("motor2", modules.launcher.launcher.launcher2.getCorrectedVelocity());


        telemetry.update();


    }

    @Override
    public void stop() {
        bec.setPosition(0);
    }

}
