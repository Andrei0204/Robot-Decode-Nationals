package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import static org.firstinspires.ftc.teamcode.ROBOT.RobotKickred.deltahhypot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Blocator.Blocator;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Launcher.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.gamepads.StickyGamepad;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Point;

@Config
public class RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    StickyGamepad stickyGamepad1, stickyGamepad2;
    RoboModules modules = null;
    public static double power = 360;
    public static double function = 24;
    public static double rpm = 60;
    public static double constant = 0.43;
    public static double constantclose = 0.37;
        public static double multfar = 0.24 ;
    public static double multpiv = 1.1   ;
    public static double multclose = 0.13;
    boolean timers;
    boolean farlaunch = false;
    boolean closelaunch=false;
    ElapsedTime backtime;
    ElapsedTime opentime;
    double deltax = 0;
    double deltay = 0;
    public static double repemekms = 0.65;
    public static double unghie =42.3;
    Point redGoal = new Point(136 * 2.54, 120 * 2.54);
    Point blueGoal = new Point(140 * 2.54, 15 * 2.54);
    public static double unghimanual=0;
    public static double unghipivot=0;
    double throwtime =-2;
    ElapsedTime timer;
    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules) {
        this.gamepad1 = gamepad1;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        backtime = new ElapsedTime();
        opentime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        stickyGamepad2 = new StickyGamepad(gamepad2);
        timer= new ElapsedTime();
        this.modules = modules;
        modules.launcher.pivot.setState(Pivot.State.AUTO);

        modules.launcher.launcher.kV= 1.29;
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
            throwtime=-2;
            modules.intake.setManualPower(0);
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
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
            if(Math.abs(deltahhypot) >170)
                modules.launcher.pivot.setangle(unghie+unghipivot);
            else
                modules.launcher.pivot.setangle(unghie-2+unghipivot);
            modules.launcher.launcher.setAuto(repemekms);
            if(modules.launcher.launcher .Motorvel()> repemekms*100-1 &&throwtime==-2)
                if(!gamepad1.isRumbling())
                    gamepad1.rumble(200);
            if(stickyGamepad1.a)
            {
                throwtime=-1;
                timer.reset();
            }
            if(throwtime>= -1) {
                modules.intake.setManualPower(1);
                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();
                }
                if (timer.milliseconds() > 100 && throwtime == 0) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);

                    throwtime++;
                    timer.reset();
                }
                if ( (timer.milliseconds() > 550 ||( modules.launcher.launcher .Motorvel()> repemekms*100-3 && timer.milliseconds()>200)) && throwtime == 1) {
                    throwtime++;
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();
                }
                if (timer.milliseconds() > 120 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();
                }
                if ((timer.milliseconds() > 450 || (modules.launcher.launcher .Motorvel()> repemekms*100-3&& timer.milliseconds()>200) ) && throwtime == 3) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();
                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime=-2;
                    modules.launcher.pivot.setangle(37);
                    timer.reset();
                }
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
        if(stickyGamepad2.b)
            unghimanual+=3;
        if(stickyGamepad2.x)
            unghimanual-=3;

        if(stickyGamepad2.a)
            unghipivot-=1;
        if(stickyGamepad2.y)
            unghipivot+=1;
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
