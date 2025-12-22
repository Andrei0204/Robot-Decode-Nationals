package org.firstinspires.ftc.teamcode.ROBOT.Modules;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
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

    public static double rpmclose = 0.65;
    public static double rpmclose2 = 0.68;
    public static double closecounter = 51;

    public static double rpmfar = 0.71;
    public static double rpmfar2 = 0.73;
    public static double farcounter = 60.5;

    public static double angl = 45;

    public static double distance = 0;


    ElapsedTime backtime;
    ElapsedTime opentime;
    ElapsedTime timer;

    public static double rpm = 0.9;

    InterpLUT close = new InterpLUT();
    InterpLUT far = new InterpLUT();
    double closelaunch = 0;
    double farlaunch = 0;
    double throwtime = -2;

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
        modules.launcher.launcher.setState(Launcher.State.TEST);

        farcounter = 63;
        closelaunch = 54;
//
//        close.add(196, 52);
//        close.add(155, 50);
//        close.add(145, 45);
//        close.add(127, 42);
////        close.add(115, 40);
        close.add(100, 36);
        close.add(110, 37.5);
        close.add(127, 38.6);
        close.add(145, 40);
        close.add(160, 40.5);
        close.add(168, 41);
        close.add(196, 41);
        close.add(240, 41);


        far.add(270, 52);
        far.add(300, 52);
        far.add(330, 50);
        far.add(360, 49);
        far.createLUT();
        close.createLUT();


    }

    public static void auto() {
        farcounter = 61.5;
        closecounter = 52;
    }

    public void updateIntake() {

        if (stickyGamepad1.right_bumper) {
            if (Math.abs(modules.intake.getManualPower()) < 0.64) {
                modules.intake.setManualPower(1);

            } else
                modules.intake.setManualPower(0);
        }
        if (gamepad1.left_bumper)
            modules.intake.setManualPower(-1);

    }


    public void updateLaunch() {

        if (stickyGamepad1.b) {
            modules.launcher.launcher.setAuto(0);
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
            throwtime = -2;
            closelaunch = 0;
            farlaunch = 0;
            modules.intake.setManualPower(0);
        }


        if (stickyGamepad1.x) {
            modules.launcher.launcher.setAuto(rpm);
            closelaunch = 1;
            modules.intake.setManualPower(0);
            farlaunch = 0;
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
        }

        if (stickyGamepad1.y) {
            modules.launcher.launcher.setAuto(rpm);
            modules.intake.setManualPower(0);
            closelaunch = 0;
            farlaunch = 1;
            modules.launcher.blocator.setState(Blocator.State.CLOSE);
        }
        if (farlaunch == 1) {
            if (modules.launcher.launcher.Motorvel() > farcounter && throwtime == -2) {
                if (!gamepad1.isRumbling())
                    gamepad1.rumble(200);
            }
            if (gamepad1.a && throwtime == -2) {
                modules.intake.setManualPower(1);
                throwtime = -1;
                timer.reset();

            }
            if (modules.launcher.launcher.Motorvel() > farcounter + 1.1) {
                modules.launcher.launcher.setAuto(farcounter / 100);
            } else if (throwtime <= 0)
                modules.launcher.launcher.setAuto(rpm);
            else
                modules.launcher.launcher.setAuto(rpm);
            if (throwtime >= -1) {

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 125 && throwtime == 0) {
                    modules.launcher.launcher.setAuto(rpm);
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }

                if (timer.milliseconds() > 170 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > farcounter && timer.milliseconds() > 200)) && throwtime == 1) {

                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 180 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 170 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > farcounter && timer.milliseconds() > 200)) && throwtime == 3) {
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


        if (closelaunch == 1) {
            if (modules.launcher.launcher.Motorvel() > closecounter && throwtime == -2) {
                if (!gamepad1.isRumbling())
                    gamepad1.rumble(200);
            }
            if (gamepad1.a && throwtime == -2) {
                modules.intake.setManualPower(1);
                throwtime = -1;
                timer.reset();

            }

            if (modules.launcher.launcher.Motorvel() > closecounter + 2.1) {
                modules.launcher.launcher.setAuto(closecounter / 100);
            } else if (throwtime <= 0)
                modules.launcher.launcher.setAuto(rpm);
            else
                modules.launcher.launcher.setAuto(rpm);
            if (throwtime >= -1) {

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 125 && throwtime == 0) {
                    modules.launcher.launcher.setAuto(rpm);
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }

                if (timer.milliseconds() > 170 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 170)) && throwtime == 1) {

                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 180 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 170 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 170)) && throwtime == 3) {
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

    public void pivotupdate() {
        if (distance < 240 && distance > 100)
            modules.launcher.pivot.setangle(close.get(distance));
        else if (distance > 270 && distance < 360)
            modules.launcher.pivot.setangle(far.get(distance));
//        else
//            modules.launcher.pivot.setangle(angl);

        if (stickyGamepad2.b) {
            farcounter = farcounter + 0.5;

        }

        if (stickyGamepad2.x) {
            farcounter = farcounter - 0.5;
        }
        if (stickyGamepad2.y) {

            closecounter = closecounter - 0.5;
        }
        if (stickyGamepad2.a) {
            closecounter = closecounter + 0.5;
        }
    }


    @Override
    public void update() {

        modules.update();

        pivotupdate();
        updateIntake();
        updateLaunch();

        stickyGamepad1.update();
        stickyGamepad2.update();
    }
}
