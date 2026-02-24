package org.firstinspires.ftc.teamcode.ROBOT.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.ROBOT.Modules.ShooterConstants;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Spinner;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Pose;

import java.util.List;
import java.util.Locale;

@Config
@Configurable
@Autonomous
@Disabled
public class farRed extends OpMode {

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

    Pose START = new Pose(110.5, 131.6, Math.toRadians(0));
    Pose LAUNCH = new Pose(87, 17, Math.toRadians(0));
    public static Pose PRE1 = new Pose(127.5, 30, Math.toRadians(-40));
    public static Pose PRE2 = new Pose(127.5, 4, Math.toRadians(-20));

    public static Pose PRE3 = new Pose(133.5, 4, Math.toRadians(20));

    //    Pose PRE1 = new Pose(131, 7.5,Math.toRadians(-0));
//    Pose PRE2 = new Pose(135, 7.5, Math.toRadians(0));
//    Pose PRE3 = new Pose(135, 7.5, Math.toRadians(0));
    public static Pose LINIA3 = new Pose(97, 39, Math.toRadians(0));
    Pose LINIA3C = new Pose(133, 30.3, Math.toRadians(0));
    //    Pose GATE2 = new Pose(133, 52, Math.toRadians(45));
    //    Pose GATE2 = new Pose(135, 46.8, Math.toRadians(55));
    Pose PARK = new Pose(108, 17, Math.toRadians(0));
    public static Pose COLLECT = new Pose(129, 15, Math.toRadians(-10));
    public static Pose COLLECT2 = new Pose(129, 10, Math.toRadians(-20));
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

        follower.setTarget(new Pose(0, 0, Math.toRadians(90)));
        control.red = 1;
        control.auto = 0;
        ShooterConstants.restartpos();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down) {
            modules.spinner.spinner.resetEncoder();
        }
    }


    @Override
    public void start() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 79, 7.5, AngleUnit.DEGREES, 0));
        modules.spinner.setRunMode(Spinner.EXTRUNMODE.PID);
        modules.spinner.setState(Spinner.State.TEST);
        modules.intake.setState(Intake.State.MANUAL);
        modules.intake.setManualPower(0);
        modules.spinner.setAuto(0);
        control.setSwm(0);
        timer.reset();
//        modules.sensorstart();
    }

    @Override
    public void loop() {
        Pose2D robopos = odo.getPosition();
        Pose localization = new Pose(robopos.getX(DistanceUnit.INCH), robopos.getY(DistanceUnit.INCH), robopos.getHeading(AngleUnit.RADIANS));

        switch (cases) {
            case GOTOPREANDLAUNCH:
                follower.setTarget(LAUNCH);
                if (timer.milliseconds() > 1600) {
                    if (follower.reachedTargetAndStopped(6)) {
                        if (shooting == 1) {
                            Launch();
                        }
                    }
                    if (follower.reachedTarget(6) && shooting == 0) {
                        shooting = 1;
                        shootingcase = 0;
                        follower.setTarget(PRE1, 2000);
                        follower.overallmulti = 1;
                        modules.intake.setManualPower(-1);
                        setCases(CASES.GOTOCOLLECT);
                    }
                } else shootingtimer.reset();
                break;
            case GOTOCOLLECT:
                if (follower.reachedTarget(12) || follower.timeout()) {
                    follower.overallmulti = 0.9;
                    follower.setTarget(PRE2);
                    timer.reset();
                    setCases(CASES.PRECOLLECT);
                }
                break;
            case PRECOLLECT:
                if (timer.milliseconds() > 600) {
                    follower.setTarget(PRE3);
                    timer.reset();
                    setCases(CASES.COLLECT);
                }
                break;
            case COLLECT:
                if (timer.milliseconds() > 1000) {
                    follower.overallmulti = 1;
                    follower.setTarget(LAUNCH);
                    setCases(CASES.GOTOLAUNCH);
                }
                break;
            case GOTOLAUNCH:
                if (localization.getX() < 90) {
                    modules.intake.setManualPower(1);
                }
                if (follower.reachedTargetAndStopped(6) && localization.getX() < 90) {
                    if (shooting == 1) {
                        Launch();
                    }
                } else shootingtimer.reset();
                if (follower.reachedTarget(6) && shooting == 0) {
                    shooting = 1;
                    shootingcase = 0;
                    follower.setTarget(LINIA3);
                    follower.overallmulti = 1;
                    modules.intake.setManualPower(-1);
                    setCases(CASES.GOTOTHIRD);
                }
                break;
            case GOTOTHIRD:
                follower.setTarget(LINIA3);
                modules.intake.setManualPower(-1);
                if (follower.reachedTarget(6) && follower.reachedHeading(Math.toRadians(10))) {

                    setCases(CASES.COLLECTTHIRD);
                }
                break;
            case COLLECTTHIRD:
                modules.intake.setManualPower(-1);
                follower.setTarget(LINIA3C);
                follower.overallmulti = 0.4;
                if (follower.reachedTarget(6)) {

                    follower.overallmulti = 0.8;
                    follower.setTarget(LAUNCH, 8000);

                    setCases(CASES.LAUNCHTHIRD);
                }
                break;
            case LAUNCHTHIRD:
                if (localization.getX() < 115) {
                    modules.intake.setManualPower(1);
                }
                if (follower.reachedTargetAndStopped(6) || follower.timeout()) {
                    if (shooting == 1) {
                        Launch();
                    }
                } else shootingtimer.reset();
                if ((follower.reachedTarget(6) || follower.timeout()) && shooting == 0) {
                    shooting = 1;
                    shootingcase = 0;
                    follower.setTarget(LINIA3);
                    follower.overallmulti = 1;
                    modules.intake.setManualPower(-1);
                    setCases(CASES.COLLECTMORE);
                    timer.reset();
                }
                break;
            case COLLECTMORE:
                if(cycles==1)
                    follower.setTarget(COLLECT, 3000);
                else
                    follower.setTarget(COLLECT2, 3000);
                if ((follower.reachedTarget(4) && timer.milliseconds() > 1000) || follower.timeout()) {
                    modules.intake.setManualPower(1);
                    setCases(CASES.LAUNCHCYCLE);
                    follower.setTarget(LAUNCH, 8000);

                } else
                    timer.reset();
                break;
            case LAUNCHCYCLE:
                if (localization.getX() < 115) {
                    modules.intake.setManualPower(1);
                }
                if (follower.reachedTargetAndStopped(6) || follower.timeout()) {
                    if (shooting == 1) {
                        Launch();
                    }
                } else shootingtimer.reset();
                if ((follower.reachedTarget(6) || follower.timeout()) && shooting == 0) {
                    shooting = 1;
                    shootingcase = 0;
                    follower.setTarget(LINIA3);
                    follower.overallmulti = 1;
                    modules.intake.setManualPower(-1);
                    setCases(CASES.COLLECTMORE);
                    cycles++;
                    if (cycles > 2)
                        setCases(CASES.PARK);
                    timer.reset();
                }
                break;
            case PARK:
                follower.setTarget(PARK);
        }

        if (localization.getY() < 125)
            control.UpLaunch();
        follower.update();
        odo.update();
        modules.update();
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("target", follower.getTarget());
        telemetry.addData("target", cases);
        telemetry.addData("target", shooting);
        telemetry.addData("target", shootingcase);
        telemetry.update();
    }

    public void setCases(CASES cases) {
        this.cases = cases;
    }

    public enum CASES {
        GOTOPREANDLAUNCH,
        GOTOCOLLECT,
        COLLECT,
        GOTOLAUNCH,
        PRECOLLECT,
        COLLECTMORE,
        LAUNCHCYCLE,
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
            if (current.milliseconds() > 150) {
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
                    shootingcase++;
                    break;
                case 1:

                    if (shootingtimer.milliseconds() > 180) {
                        modules.spinner.setAuto(220);
                        shootingcase++;
                        shootingtimer.reset();
                    }
                    break;
                case 2:

                    if (shootingtimer.milliseconds() > 290) {
                        modules.spinner.setAuto(380);
                        if (modules.spinner.getRealCurrentPosition() > 360)
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



