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

@Config
public class RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    StickyGamepad stickyGamepad1, stickyGamepad2;
    RoboModules modules = null;
    public static double power = 360;
    public static double termenliberfar = 46;
    public static double angleconstantfar = 46.5;
    public static double constant = 0.37;
    public static double multpiv = 1.44   ;
    public static double multfar = 0.26;
    public static double intaik = 1;
    public static double launchfailsafe = 0.65;
    public static double launchfailsafeclose = 0.55 ;
    boolean timers;
    boolean farlaunch = false;
    boolean closelaunch = false;
    ElapsedTime backtime;
    ElapsedTime opentime;
    public static double repemekms = 0.525;
    public static double unghie = 42.6;
    public static double unghimanual = 0;
    public static double unghipivot = 0;
    double throwtime = -2;
    ElapsedTime timer;

    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules) {
        this.gamepad1 = gamepad1;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        backtime = new ElapsedTime();
        opentime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        stickyGamepad2 = new StickyGamepad(gamepad2);
        timer = new ElapsedTime();
        this.modules = modules;
        modules.launcher.pivot.setState(Pivot.State.AUTO);
       repemekms = 0.525;


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
            if (Math.abs(modules.intake.getManualPower()) < 0.64)
                modules.intake.setManualPower(1);
            else
                modules.intake.setManualPower(0);
        }

    }


    public void updateLaunch() {
        if (modules.launcher.launcher.getTarget() == 0)
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
        if (stickyGamepad1.y) {
            modules.launcher.launcher.setState(Launcher.State.TEST);
            farlaunch = true;
            closelaunch = false;
            modules.intake.setManualPower(0);

        }


        if (farlaunch) {
//            if (stickyGamepad1.a) {
//                if (modules.launcher.blocator.getState() == Blocator.State.CLOSE) {
//                    modules.launcher.blocator.setState(Blocator.State.OPEN);
//                } else


//            if (stickyGamepad1.a) {
//                throwtime = -1;
//                modules.intake.setManualPower(1);
//                timer.reset();
//            }
////
            modules.launcher.launcher.setAuto(340.0 / 360 * multfar + constant);
            if(stickyGamepad1.a)
            {
                throwtime = -1;
                modules.intake.setManualPower(intaik);

            }



            if (modules.launcher.launcher.Motorvel() > launchfailsafe * 100  && throwtime == -2) {
                if (!gamepad1.isRumbling())
                    gamepad1.rumble(200);

            }

            if (modules.launcher.launcher.Motorvel() > 42) {
                if (Math.abs(modules.launcher.pivot.getCurrentPosition() - multpiv * modules.launcher.launcher.Motorvel() - termenliberfar) > 0.03 && modules.launcher.launcher.getAmps() < 1.5)
                    modules.launcher.pivot.setangle(multpiv * modules.launcher.launcher.Motorvel() - termenliberfar);
            }
            else
                modules.launcher.pivot.setangle(angleconstantfar);

            if (throwtime >= -1) {

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();
                }
                if (timer.milliseconds() > 110 && throwtime == 0) {

                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();
                }
                if (timer.milliseconds() > 300 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > launchfailsafe * 100 - 1 && timer.milliseconds() > 250)) && throwtime == 1) {
                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 110 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 350 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > launchfailsafe * 100 - 1 && timer.milliseconds() > 250)) && throwtime == 3) {
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime = -2;
                    timer.reset();
                }
            }

        }


        if (stickyGamepad1.b) {
            modules.launcher.launcher.setState(Launcher.State.TEST);
            modules.launcher.launcher.setAuto(0);
            farlaunch = false;
            closelaunch = false;
            throwtime = -2;
            modules.intake.setManualPower(0);
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
        }
        if (gamepad1.x) {
            modules.launcher.launcher.setState(Launcher.State.TEST);
            farlaunch = false;
            closelaunch = true;
            modules.intake.setManualPower(0);
        }
        if (closelaunch) {
            if (Math.abs(deltahhypot) > 185)
                modules.launcher.pivot.setangle(unghie + unghipivot);
            else
                modules.launcher.pivot.setangle(unghie - 2 + unghipivot);
            modules.launcher.launcher.setAuto(repemekms);


            if (modules.launcher.launcher.Motorvel() > launchfailsafeclose * 100 - 2.5 && throwtime == -2)
                if (!gamepad1.isRumbling())
                    gamepad1.rumble(200);
//            if (stickyGamepad1.a) {
//                if (modules.launcher.blocator.getState() == Blocator.State.CLOSE) {
//                    modules.launcher.blocator.setState(Blocator.State.OPEN);
//
//                } else{
//                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
//                    modules.intake.setManualPower(0);
//                }
//            }

            if (stickyGamepad1.a) {
                throwtime = -1;
                modules.intake.setManualPower(1);
                timer.reset();
            }
//
            if (throwtime >= -1) {

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
                if (timer.milliseconds() > 120 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 600 || (modules.launcher.launcher.Motorvel() > launchfailsafeclose * 100 -2 && timer.milliseconds() > 250)) && throwtime == 1) {
                    modules.intake.setManualPower(1);
                    throwtime++;
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 130 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 100 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 600 || (modules.launcher.launcher.Motorvel() > launchfailsafeclose * 100 - 2 && timer.milliseconds() > 200)) && throwtime == 3) {
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime = -2;
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
        if (stickyGamepad2.b)
            repemekms += 0.01;
        if (stickyGamepad2.x)
            repemekms -= 0.01;

        if (stickyGamepad2.a)
            unghipivot -= 1;
        if (stickyGamepad2.y)
            unghipivot += 1;
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
