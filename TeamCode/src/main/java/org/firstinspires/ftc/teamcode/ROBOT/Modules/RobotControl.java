package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Spinner;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.gamepads.StickyGamepad;

@Config
public class RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    public RoboModules modules = null;
    ElapsedTime backtime;
    ElapsedTime opentime;
    ElapsedTime timer;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu
    public double flywheelSpeed = 0;
    public static double far = 0;
    public static double rec = 1.11;
    public static double tresh = 0;


    public static int farcases = 0;
    public static double counter = 270;
    public static double farthrow = 0;
    int spin = 0;



    private final GamepadManager g1Manager = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager g2Manager = PanelsGamepad.INSTANCE.getSecondManager();


    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules, GoBildaPinpointDriver odo) {
        this.gamepad1 = gamepad1;
        backtime = new ElapsedTime();
        opentime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        timer = new ElapsedTime();
        this.modules = modules;
        this.odo = odo;
        g1Manager.asCombinedFTCGamepad(gamepad1);
        g2Manager.asCombinedFTCGamepad(gamepad2);

    }

    private void UpIn() {

        if (gamepad1.leftBumperWasPressed())
            modules.intake.setManualPower(0);
        if (gamepad1.rightBumperWasPressed())
            modules.intake.setManualPower(-1);
        if (gamepad1.dpadUpWasPressed())
            modules.intake.setManualPower(1);
    }

    public void UpSpin() {

        if (gamepad1.options) {
            modules.spinner.spinner.resetEncoder();
        }

        if (gamepad1.bWasPressed()) {
            if (spin < 0)
                modules.spinner.setAuto((modules.spinner.getRealCurrentPosition() + 120));
            else
                modules.spinner.setAuto((modules.spinner.getRealCurrentPosition() + 110));
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.TEST);
        }
        if (gamepad1.dpad_down) {
            spin = 0;
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.START);
        }


        if (gamepad1.right_trigger > 0.1) {
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            modules.spinner.setManualPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            modules.spinner.setManualPower(-gamepad1.left_trigger);
        } else {

//            modules.spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            modules.spinner.setManualPower(0);

        }


        if (gamepad1.dpadLeftWasPressed()) {
            farcases=0;
            farthrow = 1;
            modules.spinner.setAuto(120);
            timer.reset();
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.TEST);
        }
        if (farthrow == 1) {

            switch (farcases) {
                case 0:

                    farcases++;
                    break;

                case 1:

                    if(timer.milliseconds()>counter ){
                        modules.spinner.setAuto(220);
                        farcases++;
                        timer.reset();
                    }
                    break;
                case 2:

                    if (timer.milliseconds() > counter+50) {
                        modules.spinner.setAuto(320);
                        farcases++;
                        farthrow = 0;
                        farcases = 0;
                    }
                    break;

            }

        }
//        if(gamepad2.leftBumperWasPressed()) {
//            if (spin <= -2)
//                spin = 2;
//            else
//                spin--;
//
//            modules.spinner.counter(spin);
//        }else if(gamepad2.rightBumperWasPressed())
//        {
//            if (spin >= 2)
//                spin = -2;
//            else
//                spin++;
//            modules.spinner.counter(spin);
//        }
    }

    public void UpLaunch() {

        Vector robotToGoalVector = new Vector(new Pose(ShooterConstants.GOAL_POS_RED.getX() - odo.getPosX(DistanceUnit.INCH), ShooterConstants.GOAL_POS_RED.getY() - odo.getPosY(DistanceUnit.INCH)));
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

//calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(30),
                Math.toRadians(65));

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        if (far == 1) {
            if (modules.launcher.Motorvel() < modules.launcher.getLaunchtoRpm(flywheelSpeed) * 100 - 1)
                modules.pivot.setangle(90 - Math.toDegrees(hoodAngle) - rec * (modules.launcher.getLaunchtoRpm(flywheelSpeed) * 100 - modules.launcher.Motorvel()));
            else
                modules.pivot.setangle(90 - Math.toDegrees(hoodAngle));
        } else
            modules.pivot.setangle(90 - Math.toDegrees(hoodAngle));
        if (modules.launcher.Motorvel() > modules.launcher.getLaunchtoRpm(flywheelSpeed) * 100 + 1) {
            modules.launcher.setAuto(modules.launcher.getLaunchtoRpm(flywheelSpeed) - 0.03);
        } else
            modules.launcher.LaunchtoRpm(flywheelSpeed);

        double deltax = +odo.getPosX(DistanceUnit.INCH) - ShooterConstants.GOAL_POS_RED.getX();
        double deltay = -odo.getPosY(DistanceUnit.INCH) + ShooterConstants.GOAL_POS_RED.getY();

        double impartire = deltay / deltax;
        double alpha = -Math.atan(impartire);

        double anglelocal = odo.getHeading(AngleUnit.RADIANS) - alpha;
        double anglenorm = Math.toDegrees(normalizaion(anglelocal));
        if (anglenorm <= -140)
            anglenorm += 360;

        modules.lTur.setangle(+anglenorm);


    }

    public double normalizaion(double angle) {
        return (angle + Math.PI) % (2 * Math.PI) - Math.PI;
    }

    @Override
    public void update() {
        UpIn();
        UpSpin();
        UpLaunch();
    }
}
