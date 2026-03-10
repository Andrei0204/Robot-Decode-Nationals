package org.firstinspires.ftc.teamcode.ROBOT.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Follower;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Intake;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Spinner;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Pose;

import java.util.List;
import java.util.Locale;

@Config
@Configurable
@Autonomous
public class closeReds extends OpMode {

    ElapsedTime timer;
    int once = 0;
    int shootingcase = 0;
    int shooting = 1;
    int isstuck = 0;
    double stuckpos = 0;
    int cycles = 1;
    public static double distance = 100;
    ElapsedTime shootingtimer;
    ElapsedTime isstucktimer;
    ElapsedTime current;

    public static double x = -6, y = -148;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu
    Follower follower;
    RoboModules modules;
    RobotControl control;
    CASES cases = CASES.GOTOPREANDLAUNCH;
    double auto = 0;

    Pose START = new Pose(110.5, 134.6, Math.toRadians(0));
    Pose LAUNCH = new Pose(90, 80, Math.toRadians(0));
    Pose LAUNCH2 = new Pose(81, 95, Math.toRadians(45));
    Pose LINIA1 = new Pose(101.0, 75.5, Math.toRadians(0));
    Pose LINIA1C = new Pose(136, 84, Math.toRadians(0));
    Pose LINIA2 = new Pose(101.0, 50, Math.toRadians(0));
    Pose LINIA2C = new Pose(137, 50, Math.toRadians(0));
    Pose LINIA3 = new Pose(101.0, 26, Math.toRadians(0));
    Pose LINIA3C = new Pose(137, 26, Math.toRadians(0));
    Pose GATE1 = new Pose(136, 51, Math.toRadians(20));
    Pose GATE2 = new Pose(136, 51, Math.toRadians(20));

    //    Pose GATE1 = new Pose(132, 52.5, Math.toRadians(31));
    Pose GATEOTW = new Pose(100, 47, Math.toRadians(20));
    //    Pose GATE2 = new Pose(133, 52, Math.toRadians(45));
    //    Pose GATE2 = new Pose(135, 46.8, Math.toRadians(55));
//    Pose PARK = new Pose(108.0, 79.0, Math.toRadians(10));
//
//    Pose START = new Pose(110.5, 131.6, Math.toRadians(0));
//    Pose LAUNCH = new Pose(91, 81.1, Math.toRadians(0));
//    Pose LINIA1 = new Pose(101.0, 81.5, Math.toRadians(0));
//    Pose LINIA1C = new Pose(138, 81.5, Math.toRadians(0));
//    Pose LINIA2 = new Pose(101.0, 56.5, Math.toRadians(0));
//    Pose LINIA2C = new Pose(139, 56.5, Math.toRadians(0));
//    Pose LINIA3 = new Pose(101.0, 32, Math.toRadians(0));
//    Pose LINIA3C = new Pose(137, 32, Math.toRadians(0));
//    Pose GATE1 = new Pose(131, 69, Math.toRadians(0));
//    Pose GATE2 = new Pose(134.6, 47, Math.toRadians(30));
//    //    Pose GATE1 = new Pose(132, 52.5, Math.toRadians(31));
//    Pose GATEOTW = new Pose(100, 54, Math.toRadians(0));
//    //    Pose GATE2 = new Pose(133, 52, Math.toRadians(45));
//    //    Pose GATE2 = new Pose(135, 46.8, Math.toRadians(55));
//    Pose PARK = new Pose(108.0, 79.0, Math.toRadians(10));
    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shootingtimer = new ElapsedTime();
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        odo.setOffsets(x, y, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        follower = new Follower(hardwareMap, odo);
        timer = new ElapsedTime();
        current = new ElapsedTime();
        isstucktimer = new ElapsedTime();
        modules = new RoboModules(hardwareMap);
        control = new RobotControl(gamepad1, gamepad2, modules, odo);
        modules.sensorstart();
        follower.setTarget(new Pose(0, 0, Math.toRadians(-90)));
        control.red = 1;
    }

    @Override
    public void init_loop() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 110.5, 134.6, AngleUnit.DEGREES, -90));
        if (gamepad1.dpad_down) {
            modules.spinner.spinner.resetEncoder();
        }
        control.UpLaunchnolauncher();
        modules.lTur.update();
//        modules.launcher.setAuto(0);
    }


    @Override
    public void start() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 110.5, 131.6, AngleUnit.DEGREES, -90));
        modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
        modules.spinner.setState(Spinner.State.TEST);
        modules.intake.setState(Intake.State.MANUAL);
        modules.intake.setManualPower(0);
        modules.spinner.setAuto(0);
        control.setSwm(1);
        control.swmtur = 1;
        control.auto=1;
//        modules.sensorstart();
    }

    @Override
    public void loop() {
        odo.update();
        Pose2D robopos = odo.getPosition();
        Pose localization = new Pose(robopos.getX(DistanceUnit.INCH), robopos.getY(DistanceUnit.INCH), robopos.getHeading(AngleUnit.RADIANS));

        switch (cases) {
            case GOTOPREANDLAUNCH:
                follower.setTarget(LAUNCH, 3000);
                if (localization.getY() < 95) {
                    if (shooting == 1)
                        Launch();
                } else shootingtimer.reset();
                if ((follower.reachedTarget(6) && shooting == 0) || (follower.timeout())) {
                    shooting = 1;
                    shootingcase = 0;
                    setCases(CASES.GOTOSECOND);
                }
                break;


            case GOTOSECOND:
                follower.setTarget(LINIA2,1600);
                modules.intake.setManualPower(-1);
                if ((follower.reachedTarget(9) && follower.reachedHeading(Math.toRadians(10)) ||follower.timeout()))
                    setCases(CASES.COLLECTSECOND);
                break;
            case COLLECTSECOND:
                follower.setTarget(LINIA2C,2000);
                follower.overallmulti = 0.8;
                if (follower.reachedTarget(8) ||follower.timeout()) {
                    setCases(CASES.LAUNCHSEC);
                    follower.overallmulti = 1;
                    follower.setTarget(LAUNCH,2000);

                    setCases(CASES.LAUNCHSEC);
                }
                break;
            case LAUNCHSEC:
                if (follower.reachedTarget(10)||follower.timeout()) {
                    modules.intake.setManualPower(1);
                    if (shooting == 1)
                        Launch();
                } else shootingtimer.reset();
                if (shooting == 0) {
                    shootingcase = 0;
                    shooting = 1;
                    if (cycles <= 2)
                        setCases(CASES.TOGATE);
                    else
                        setCases(CASES.GOTOFIRST);
                }
                break;
            case TOGATE:

//                control.setSwm(0);
                follower.setTarget(GATEOTW,2000);
                modules.intake.setManualPower(-1);
                if (follower.reachedTarget(15)||follower.timeout()) {
                    follower.overallmulti = 0.6;

                    follower.setTarget(new Pose(GATE1.getX()+(cycles-1), GATE1.getY() - (cycles - 1) * 2, GATE1.getHeading()),1500);
                    timer.reset();
                    setCases(CASES.ARRIVETOGATE);
                }
                break;
            case ARRIVETOGATE:
                if (follower.reachedTargetOrStopped(12)||follower.timeout()) {
                    if(timer.milliseconds()>350 && cycles==2){
                        follower.overallmulti = 1;
                        follower.setTarget(GATE2,1000);
                        setCases(CASES.COLLECTGATE);
                        timer.reset();
                    }
                    else if(cycles!=2)
                    {
                        follower.overallmulti = 1;
                        follower.setTarget(GATE2,1000);
                        setCases(CASES.COLLECTGATE);
                        timer.reset();
                    }
                }
                else timer.reset();
                break;
            case COLLECTGATE:
                if (timer.milliseconds() > 1700) {
                    modules.intake.setManualPower(1);
                    follower.setTarget(LAUNCH,2400);
                    setCases(CASES.LAUNCHSEC);
                    cycles++;
                }
                break;
            case GOTOFIRST:

                modules.intake.setManualPower(-1);
                setCases(CASES.COLLECTFIRST);
                break;
            case COLLECTFIRST:

                follower.setTarget(LINIA1C,2000);
                follower.overallmulti = 0.4;

                if (follower.reachedTarget(12)||follower.timeout()) {
                    follower.overallmulti = 1;
                    follower.setTarget(LAUNCH,2000);

                    setCases(CASES.LAUNCHFIRST);
                }
                break;
            case LAUNCHFIRST:
                follower.overallmulti = 1;
                if (localization.getX() < 110 )
                    modules.intake.setManualPower(1);
                if (follower.reachedTarget(10)||follower.timeout()) {

                    if (shooting == 1)
                        Launch();
                } else shootingtimer.reset();
                if (shooting == 0) {
                    shootingcase = 0;
                    shooting = 1;
//                    setCases(CASES.PARK);
                    setCases(CASES.GOTOTHIRD);
                }
                break;
            case GOTOTHIRD:
                follower.setTarget(LINIA3,3000);
                modules.intake.setManualPower(-1);
                if ((follower.reachedTarget(9) && follower.reachedHeading(Math.toRadians(10)))||follower.timeout()) {

                    setCases(CASES.COLLECTTHIRD);
                }
                break;
            case COLLECTTHIRD:
                modules.intake.setManualPower(-1);
                follower.setTarget(LINIA3C,2000);
                follower.overallmulti = 0.6;
                if (follower.reachedTarget(6)||follower.timeout()) {

                    follower.overallmulti = 1;
                    follower.setTarget(LAUNCH2,3500);

                    setCases(CASES.LAUNCHTHIRD);
                }
                break;
            case LAUNCHTHIRD:
                if (localization.getX() < 132)
                    modules.intake.setManualPower(1);
                if (follower.reachedTarget(10)|| follower.timeout()) {

                    if (shooting == 1)
                        Launch();
                } else shootingtimer.reset();
                if (shooting == 0) {
                    shootingcase = 0;
                    shooting = 1;
                    setCases(CASES.PARK);
                }
                break;
            case PARK:
//                follower.setTarget(PARK);
                break;

        }

//        if (localization.getY() < 125)
            control.UpLaunch();
        follower.update();

        modules.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("target", follower.getTarget());
        telemetry.addData("target", cases);
        telemetry.addData("target", shooting);
        telemetry.addData("target", shootingcase);
        telemetry.addData("state", modules.spinner.getAuto());
        telemetry.addData("state", modules.spinner.getState()   );
        telemetry.update();
    }

    public void setCases(CASES cases) {
        this.cases = cases;
    }

    public enum CASES {
        GOTOPREANDLAUNCH,
        GOTOSECOND,
        COLLECTSECOND,
        LAUNCHSEC,
        TOGATE,
        ARRIVETOGATE, COLLECTGATE, GOTOFIRST, COLLECTFIRST, LAUNCHFIRST, PARK, GOTOTHIRD, COLLECTTHIRD, LAUNCHTHIRD;
    }

    public void Launch() {
//        if (isstuck == 0) {
//            if (modules.spinner.spinner.motor.motor.getCurrent(CurrentUnit.AMPS) > 8) {
//                if (current.milliseconds() > 230) {
//                    isstuck = 1;
//                    auto = modules.spinner.getAuto();
//                    modules.spinner.setAuto(modules.spinner.getRealCurrentPosition() - 20);
//                    current.reset();
//                }
//            } else {
        if (modules.spinner.spinner.motor.motor.getCurrent(CurrentUnit.AMPS) > 7 && isstuck != 1) {
            if (current.milliseconds() > 300) {
                stuckpos = modules.spinner.getRealCurrentPosition();
                isstucktimer.reset();
                isstuck = 1;
            }
        } else
            current.reset();
        if (isstuck == 1) {

            modules.spinner.setAuto(stuckpos - 80);
            if (isstucktimer.milliseconds() > 250)
                isstuck = 0;

        } else {

            switch (shootingcase) {
                case 0:
                    modules.spinner.setAuto(120);
                    shootingtimer.reset();
                    shootingcase++;
                    break;
                case 1:

                    if (shootingtimer.milliseconds() > 0) {
                        modules.spinner.setAuto(220);
                        shootingcase++;
                        shootingtimer.reset();
                    }
                    break;
                case 2:

                    if (shootingtimer.milliseconds() > 0) {
                        modules.spinner.setAuto(400);
                        if (modules.spinner.getRealCurrentPosition() > 370)
                            shootingcase++;

                    }
                    break;
                case 3:
                    modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
                    modules.spinner.setAuto(0);
                    shooting = 0;
                    shootingcase = 0;

                    break;
            }
        }
    }
//        } else {
//            if (current.milliseconds() > 300) {
//                isstuck = 0;
//                modules.spinner.setAuto(auto);
//            }
//        }
//
//    }
}



