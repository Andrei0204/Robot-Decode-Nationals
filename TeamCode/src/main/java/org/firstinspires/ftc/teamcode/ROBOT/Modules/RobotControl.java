package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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
import org.opencv.core.Mat;

@Configurable

@Config
public class  RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    public RoboModules modules = null;
    ElapsedTime backtime;
    ElapsedTime opentime;
    ElapsedTime timer;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu
    public double flywheelSpeed = 0;
    public static Vector robotToGoalVector = new Vector();
    public double counter = 0;
    public static double shooting = 0;

    public static double compensation = 1;
    public  double rec = 0.25;
    public static double recf = 0.25;
    public static double recc = 0.4;
    public static double counterf = 175;
    public static double counterc = 150;
    public static double magnitude = 110;
    public int shootingcase = 0;

    public static int swm = 0;
    public static int minus = 1;




    public  double alpha = 0;
    public  double anglelocal = 0;
    public  double anglenorm = 0;


    int spin = 0;


    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules, GoBildaPinpointDriver odo) {
        this.gamepad1 = gamepad1;
        backtime = new ElapsedTime();
        opentime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        timer = new ElapsedTime();
        this.modules = modules;
        this.odo = odo;


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

//
            modules.spinner.setManualPower(0);

        }


        if (gamepad1.dpadLeftWasPressed()) {
            shootingcase = 0;
            shooting = 1;
            modules.spinner.setAuto(120);
            timer.reset();
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.TEST);
        }
        if (shooting == 1) {

            switch (shootingcase) {
                case 0:

                    shootingcase++;
                    break;

                case 1:

                    if (timer.milliseconds() > counter) {
                        modules.spinner.setAuto(220);
                        shootingcase++;
                        timer.reset();
                    }
                    break;
                case 2:

                    if (timer.milliseconds() > counter ) {
                        modules.spinner.setAuto(320);
                        shootingcase++;
                        shooting = 0;
                        shootingcase = 0;
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

        robotToGoalVector = new Vector(new Pose(ShooterConstants.GOAL_POS_RED.getX() - odo.getPosX(DistanceUnit.INCH), ShooterConstants.GOAL_POS_RED.getY() - odo.getPosY(DistanceUnit.INCH)));
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT_CLOSE;
        double a = ShooterConstants.SCORE_ANGLE_CLOSE;
        rec=recc;
        counter=counterc;
        if(robotToGoalVector.getMagnitude()>magnitude)
        {
            counter=counterf;
            rec=recf;
            y=ShooterConstants.SCORE_HEIGHT_FAR;
            a=ShooterConstants.SCORE_ANGLE_FAR;
        }
//calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(30),
                Math.toRadians(65));

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));


        Vector robotVelocity = new Vector(new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH))) ;
        double coordinateTheta = robotVelocity.getTheta()-robotToGoalVector.getTheta();
        double parallelComponent= -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent= Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz=flywheelSpeed* Math.sin(hoodAngle);
        double time = x/(flywheelSpeed*Math.cos(hoodAngle));
        double ivr= x/time + parallelComponent;
        double nvr= Math.sqrt(ivr*ivr +perpendicularComponent *perpendicularComponent);
        double ndr= nvr*time;
        double turretVelComp=0;
        if(swm==1) {
            hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), Math.toRadians(30), Math.toRadians(65));
            flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle)-y)));
            turretVelComp= Math.atan(perpendicularComponent/ivr);

        }


        if (compensation == 1) {
            if (modules.launcher.Motorvel() < modules.launcher.getLaunchtoRpm(flywheelSpeed) * 100 - 1)
                modules.pivot.setangle(90 - Math.toDegrees(hoodAngle) - rec * (modules.launcher.getLaunchtoRpm(flywheelSpeed) * 100 - modules.launcher.Motorvel()));
            else
                modules.pivot.setangle(90 - Math.toDegrees(hoodAngle));
        } else
            modules.pivot.setangle(90 - Math.toDegrees(hoodAngle));

        modules.launcher.LaunchtoRpm(flywheelSpeed);

        alpha = robotToGoalVector.getTheta();
        anglelocal = odo.getHeading(AngleUnit.RADIANS) - alpha;
        anglenorm = Math.toDegrees(MathFunctions.normalizeAngle(anglelocal) +minus*turretVelComp);

        if (anglenorm >= 220)
            anglenorm -= 360;

        modules.lTur.setangle(+anglenorm);
//    }

    }


    @Override
    public void update() {
        UpIn();
        UpSpin();
        UpLaunch();
    }
}
//        double dis = Math.pow(realspeed, 4) -( g * ((g * x * x) + (2 * y * realspeed * realspeed)));
//        if (dis >= 0) {
//            realangle = Math.atan((realspeed * realspeed - Math.sqrt(dis) * minus) / (g * x));
//            modules.pivot.setangle(90 - Math.toDegrees(realangle));
//        } else
//            modules.pivot.setangle(90 - Math.toDegrees(hoodAngle));


//        if (modules.launcher.Motorvel() > modules.launcher.getLaunchtoRpm(flywheelSpeed) * 100 + 1) {
//            modules.launcher.setAuto(modules.launcher.getLaunchtoRpm(flywheelSpeed) - 0.03);
//        } else