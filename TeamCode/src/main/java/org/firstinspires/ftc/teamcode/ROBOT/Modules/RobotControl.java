package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.ROBOT.RobotKick.deltahhypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Blocator.Blocator;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Launcher.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Point;

import java.util.Locale;

@Config
public class RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    StickyGamepad stickyGamepad1, stickyGamepad2;
    RoboModules modules = null;
    public static double power = 360;
    public static double function = 24;
    public static double rpm = 60;
    public static double constant = 0.4;
    public static double constantclose = 0.37;
        public static double multfar = 0.23;
    public static double multpiv = 1.152   ;
    public static double multclose = 0.13;
    boolean timer;
    boolean farlaunch = false;
    boolean closelaunch=false;
    ElapsedTime backtime;
    ElapsedTime opentime;
    double deltax = 0;
    double deltay = 0;
    public double alpha = 0;
    public double truealpha = 0;
    Point redGoal = new Point(136 * 2.54, 120 * 2.54);
    Point blueGoal = new Point(140 * 2.54, 15 * 2.54);

    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules) {
        this.gamepad1 = gamepad1;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        backtime = new ElapsedTime();
        opentime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        stickyGamepad2 = new StickyGamepad(gamepad2);

        this.modules = modules;
        modules.launcher.pivot.setState(Pivot.State.AUTO);


    }


    public void updateIntake() {
        if (gamepad1.left_bumper)
            modules.intake.setManualPower(-1);

        if (farlaunch) {
            if (modules.intake.getManualPower() > 0 && modules.launcher.launcher.getTarget() != 0) {
                modules.intake.setManualPower(0.7);

            }
//            if (timer) {
//                if (backtime.milliseconds() > 300) {
//                    if (modules.intake.getManualPower() < -0.3)
//                        modules.intake.setManualPower(0.8);
//
//                }
//                if(backtime.milliseconds()>500) {
//                    timer = false;
//                    backtime.reset();
//                    modules.intake.setManualPower(0);
//                }
//            }

        if (modules.intake.getManualPower() == 0.7 && modules.launcher.launcher.getTarget() == 0) {
            modules.intake.setManualPower(1);
        }
    }
        if (stickyGamepad1.right_bumper) {
            if (Math.abs(modules.intake.getManualPower() )<0.64 )
                modules.intake.setManualPower(1);
            else
                modules.intake.setManualPower(0);
        }

    }


    public void updateLaunch() {
        if (modules.launcher.launcher.getTarget() == 0)
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
        if(farlaunch)
            if (stickyGamepad1.a) {
            if (modules.launcher.blocator.getState() == Blocator.State.CLOSE) {
                modules.launcher.blocator.setState(Blocator.State.OPEN);
            }

            else
//                if(!timer)
                  modules.launcher.blocator.setState(Blocator.State.CLOSE);
             }



        if (stickyGamepad1.y) {
            modules.launcher.launcher.setState(Launcher.State.TEST);
            farlaunch = true;
            closelaunch=false;
        }
        if (farlaunch)
            if (deltahhypot > 300)
                modules.launcher.launcher.setAuto(Math.abs(deltahhypot) / 360 * 0.18 + constant);
            else
                modules.launcher.launcher.setAuto(340.0 / 360 * multfar + constant);
        if (stickyGamepad1.b) {
            modules.launcher.launcher.setState(Launcher.State.TEST);
            modules.launcher.launcher.setAuto(0);
            farlaunch = false;
            closelaunch = false;
        }
        if (modules.launcher.launcher.Motorvel() > 53) {
            if (Math.abs(modules.launcher.pivot.getCurrentPosition() - 1.2 * modules.launcher.launcher.Motorvel() - 26) > 0.03 && modules.launcher.launcher.getAmps() < 4)
                modules.launcher.pivot.setangle(multpiv * modules.launcher.launcher.Motorvel() - function);
        } else
            modules.launcher.pivot.setangle(39);


        if(gamepad1.x) {
            modules.launcher.launcher.setState(Launcher.State.TEST);
         farlaunch=false;
            closelaunch = true;
        }
        if(closelaunch)
        {
            modules.launcher.pivot.setangle(39);
            modules.launcher.launcher.setAuto(Math.abs(deltahhypot) / 220 * multclose + constantclose);
            if(stickyGamepad1.a)
                if(modules.launcher.blocator.getState()== Blocator.State.CLOSE) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    modules.intake.setManualPower(1);
                }
                else{

                        modules.launcher.blocator.setState(Blocator.State.CLOSE);
                        modules.intake.setManualPower(0);
                    }
        }


    }

    public void turetupdate() {
//        Pose2D pos = odo.getPosition();
//        boolean isred = true;
//        if (gamepad1.dpad_left)
//            isred = true;
//        if (gamepad1.dpad_right)
//            isred = false;
//        if (isred) {
//            deltax = pos.getX(DistanceUnit.CM) - redGoal.x;
//            deltay = pos.getY(DistanceUnit.CM) - redGoal.y;
//        } else {
//            deltax = pos.getX(DistanceUnit.CM) - blueGoal.x;
//            deltay = pos.getY(DistanceUnit.CM) - blueGoal.y;
//        }
//        alpha= Math.atan2(deltay,deltax);
//        truealpha = alpha + pos.getHeading(AngleUnit.RADIANS);
//        if(gamepad1.dpad_up)
//            odo.resetPosAndIMU();
    }


    @Override
    public void update() {

        modules.update();
        stickyGamepad1.update();
        stickyGamepad2.update();
        updateIntake();
        updateLaunch();
        turetupdate();

    }
}
