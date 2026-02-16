package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;

@Configurable

@Config
public class RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    public RoboModules modules = null;
    ElapsedTime backtime;
    ElapsedTime opentime;
    public ElapsedTime shootingtimer;
    ElapsedTime reversetimer;
    ElapsedTime sensortimer;
    ElapsedTime isstucktimer;
    ElapsedTime current;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu
    public double flywheelSpeed = 0;
    public static Vector robotToGoalVector = new Vector();
    public double counter = 0;
    public static double shooting = 0;
    double stuckpos =0;
    public static double compensation = 1;
    public static double red = 1;
    public  double isstuck = 0;
    public double rec = 0.25;
    public static double recf = 0.25;
    public static double recc = 0.1;
    public static double counterf = 270;
    public static double counterc = 180;
    public static double magnitude = 110;
    public int shootingcase = 0;

    public int swm = 1;
    public int auto = 0;
    public int swmtur = 1;
    public int swmon = 1;
    public static int minus = 1;
//

    public double alpha = 0;
    public double anglelocal = 0;
    public double anglenorm = 0;


    public double[] CURRENT;
//    public static double[] slots;
//    public int swaps;
//    public static int cases=1;

    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules, GoBildaPinpointDriver odo) {
        this.gamepad1 = gamepad1;
        backtime = new ElapsedTime();
        reversetimer = new ElapsedTime();
        sensortimer = new ElapsedTime();
        opentime = new ElapsedTime();
        current = new ElapsedTime();
        isstucktimer = new ElapsedTime();
        this.gamepad2 = gamepad2;
        shootingtimer = new ElapsedTime();
        this.modules = modules;
        this.odo = odo;
        swmtur=1;
        auto=0;
    }

    public void setSwm(int swm) {
        this.swmon = swm;
    }

    private void UpIn() {

        if (gamepad1.rightBumperWasPressed()) {
            modules.intake.setManualPower(-1);

        }

        if (gamepad1.leftBumperWasPressed())
            modules.intake.setManualPower(1);
        if (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased())
            modules.intake.setManualPower(0);

        if (sensortimer.milliseconds() > 350) {
            sensortimer.reset();
            if ((modules.distance.getDetection() < 120 || modules.distancec.getDetection() < 20)
                    && (modules.distance2.getDetection() < 120 || modules.distance2c.getDetection() < 20)
                    && (modules.distance3.getDetection() < 120 || modules.distance3c.getDetection() < 20)
                    && Math.abs(modules.spinner.getRealCurrentPosition()) - 10 < 0 &&
                    !gamepad1.right_bumper) {

                reversetimer.reset();
                modules.intake.setManualPower(1);

            }
        }if(modules.intake.getManualPower()==1)
            modules.bec.setPosition(0.5);
        if (sensortimer.milliseconds() > 300 && modules.intake.getManualPower() == 1) {
            modules.intake.setManualPower(0);
            reversetimer.reset();
        }
    }

    public void UpSpin() {

        if (gamepad1.options) {
            modules.spinner.spinner.resetEncoder();
        }

        if (gamepad1.left_trigger > 0.2) {
            modules.spinner.setAuto((modules.spinner.getRealCurrentPosition() + 110));
            if (modules.spinner.getRealCurrentPosition() < -10)
                modules.spinner.setAuto((modules.spinner.getRealCurrentPosition() + 150));
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.TEST);
        }
        if (gamepad1.dpad_down) {
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.START);
        }

        if (gamepad2.right_trigger > 0.1) {
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            modules.spinner.setManualPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1) {
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.MANUAL);
            modules.spinner.setManualPower(-gamepad2.left_trigger);
        } else {

//
            modules.spinner.setManualPower(0);

        }


        if (gamepad1.right_trigger > 0.2 && shooting == 0) {
            modules.intake.setManualPower(0);
            shootingcase = 0;
            shooting = 1;
            modules.spinner.setAuto(120);
            shootingtimer.reset();
            modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
            modules.spinner.setState(Spinner.State.TEST);
        }
        if (shooting == 1) {

            if (modules.spinner.spinner.motor.motor.getCurrent(CurrentUnit.AMPS) > 7 && isstuck != 1) {
                if (current.milliseconds() > 250) {
                    stuckpos= modules.spinner.getRealCurrentPosition();
                    isstucktimer.reset();
                    isstuck = 1;
                }
            } else
                current.reset();
            if (isstuck == 1) {

                modules.spinner.setAuto(stuckpos - 80);
                if (isstucktimer.milliseconds() > 400)
                    isstuck = 0;

            } else {

                switch (shootingcase) {
                    case 0:

                        shootingcase++;
                        break;

                    case 1:

                        if (shootingtimer.milliseconds() > counter) {
                            modules.spinner.setAuto(220);
                            shootingcase++;
                            shootingtimer.reset();
                        }
                        break;
                    case 2:

                        if (shootingtimer.milliseconds() > counter) {
                            modules.spinner.setAuto(380);
                            if (modules.spinner.getRealCurrentPosition() > 370)
                                shootingcase++;

                        }
                        break;
                    case 3:
                        modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
                        modules.spinner.setState(Spinner.State.START);
                        shooting = 0;
                        shootingcase = 0;
                        break;
                }
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

//    public void sortlaunch() {
//        int opos = -1;
//
//        cases = 1;
//        switch (cases) {
//            case 1:
//                opos = 0;
//                break;
//            case 2:
//                opos = 1;
//                break;
//            case 3:
//                opos = 2;
//                break;
//            default:
//                opos = 0;
//                break;
//        }
//
//        double c1 = (modules.color.getDetection() == Color.detection.PURPLE ? 1 : 0);
//        double c2 = (modules.color2.getDetection() == Color.detection.PURPLE ? 1 : 0);
//        double c3 = (modules.color3.getDetection() == Color.detection.PURPLE ? 1 : 0);
//        slots = new double[]{c1, c2, c3};
//
//        int zpos = -1;
//        for (int i = 0; i <= 2; i++) {
//            if (slots[i] == 0)
//                zpos = i;
//        }swaps=Math.abs((opos-zpos)%3);
//        if(swaps==1)
//        {
//            swaps=-300;
//        }
//       else  if(swaps==2)
//        {
//            swaps=-190;
//        }
//        else
//            swaps=0;
//        modules.spinner.setState(Spinner.State.TEST);
//        modules.spinner.setAuto( swaps);
//
//
//    }

    public void UpLaunch() {


        if(red==1) {
            if(auto==0)
             robotToGoalVector = new Vector(new Pose(ShooterConstants.GOAL_POS_RED.getX() - odo.getPosX(DistanceUnit.INCH), ShooterConstants.GOAL_POS_RED.getY() - odo.getPosY(DistanceUnit.INCH)));
            else
                robotToGoalVector = new Vector(new Pose(ShooterConstants.GOAL_POS_RED_AUTO.getX() - odo.getPosX(DistanceUnit.INCH), ShooterConstants.GOAL_POS_RED_AUTO.getY() - odo.getPosY(DistanceUnit.INCH)));

        }else {
            if(auto==0)
                robotToGoalVector = new Vector(new Pose(ShooterConstants.GOAL_POS_BLUE.getX() - odo.getPosX(DistanceUnit.INCH), ShooterConstants.GOAL_POS_BLUE.getY() - odo.getPosY(DistanceUnit.INCH)));
            else
                robotToGoalVector = new Vector(new Pose(ShooterConstants.GOAL_POS_BLUE_AUTO.getX() - odo.getPosX(DistanceUnit.INCH), ShooterConstants.GOAL_POS_BLUE_AUTO.getY() - odo.getPosY(DistanceUnit.INCH)));


        }
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT_CLOSE;
        double a = ShooterConstants.SCORE_ANGLE_CLOSE;
        rec = recc;
        swm = swmon;
        counter = counterc;
        if (robotToGoalVector.getMagnitude() > magnitude) {
            swm = 0;
            counter = counterf;
            rec = recf;
            y = ShooterConstants.SCORE_HEIGHT_FAR;
            a = ShooterConstants.SCORE_ANGLE_FAR;
        }
//calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Math.toRadians(30),
                Math.toRadians(55));

        flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));


        Vector robotVelocity = new Vector(new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH)));
        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;
        double turretVelComp = 0;
        if (swm == 1) {
            hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), Math.toRadians(30), Math.toRadians(55));
            flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
            if(swmtur==1)
                turretVelComp = Math.atan(perpendicularComponent / ivr);

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
        anglenorm = Math.toDegrees(MathFunctions.normalizeAngle(anglelocal) + minus * turretVelComp);

        if (anglenorm >= 220)
            anglenorm -= 360;

        modules.lTur.setangle(+anglenorm);
//    }

//        if (gamepad1.dpadLeftWasPressed()) {
//            sortlaunch();
//        }
    }


    @Override
    public void update() {
        modules.bec.setPosition(1);
        UpIn();
        UpSpin();
        UpLaunch();
    }

    public void Launch() {
        switch (shootingcase) {
            case 0:

                shootingcase++;
                break;

            case 1:

                if (shootingtimer.milliseconds() > counter) {
                    modules.spinner.setAuto(220);
                    shootingcase++;
                    shootingtimer.reset();
                }
                break;
            case 2:

                if (shootingtimer.milliseconds() > counter) {
                    modules.spinner.setAuto(380);
                    if (modules.spinner.getRealCurrentPosition() > 370)
                        shootingcase++;

                }
                break;
            case 3:
                modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
                modules.spinner.setState(Spinner.State.START);
                shooting = 0;
                shootingcase = 0;

                break;
        }

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