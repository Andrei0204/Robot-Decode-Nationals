package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import static org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret.TICKSDEG;
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret.autopose;
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl.closecounter;
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl.rpmclose;
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl.rpmclose2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Follower;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Blocator.Blocator;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Launcher.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Position;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Point;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Pose;

import java.util.Locale;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous
@Config
public class AutoCloseRed extends OpMode {
    Position pose;
    boolean f = true, l = false;
    RoboModules modules;
    ElapsedTime timer;
    ElapsedTime movement;
    Point redGoal = new Point(136 * 2.54, -136 * 2.54);
    Point blueGoal = new Point(140 * 2.54, -8.5 * 2.54);
    boolean isred = true;
    GoBildaPinpointDriver odo = null;
    Follower follower;
    int cases = 1;
    int throwtime = -1;
    public static double rpm = 0.51;
    public static double angl = 45.5;

    Limelight3A limelight;
    LLResult result=null;
    @Override
    public void init() {


// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        timer = new ElapsedTime();
        movement = new ElapsedTime();
        odo.setOffsets(54, -164.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        follower = new Follower(hardwareMap, odo);
        modules = new RoboModules(hardwareMap);

        modules.launcher.turret.setState(Turret.State.TEST);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();
    }


    @Override
    public void init_loop() {
        if (gamepad1.dpad_up)
            modules.launcher.turret.turret.resetEncoder();
        modules.launcher.turret.setAuto(0 * TICKSDEG);

    }

    @Override
    public void start() {
        modules.update();
        odo.setPosition(new Pose2D(DistanceUnit.CM, 310, -310, AngleUnit.RADIANS, -0.78));
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */

    @Override
    public void loop() {
        modules.update();
        follower.update();
        if(Launcher.getAutopose()!=0) {
            if (modules.launcher.launcher.Motorvel() > closecounter + 2.1) {
                modules.launcher.launcher.setAuto(closecounter / 100);
            } else if (throwtime <= 0)
                modules.launcher.launcher.setAuto(rpmclose);
            else
                modules.launcher.launcher.setAuto(rpmclose2);
        }
        switch (cases) {
            case 1: //Gotoprelaunch

                modules.launcher.pivot.setState(Pivot.State.AUTO);
                modules.launcher.launcher.setState(Launcher.State.TEST);
                modules.launcher.turret.setState(Turret.State.TEST);


                if(!follower.reachedTarget(6))
                    timer.reset();
                follower.setTarget(new Pose(195, -230, Math.toRadians(-90)));
                modules.launcher.pivot.setangle(40.5);
                if (follower.localization.getX() < 290) {
                    modules.launcher.turret.setAuto((angl + 1) * TICKSDEG);
                }
                if (follower.localization.getX() < 220) {
                    modules.launcher.launcher.setAuto(rpmclose);
                }


                if(follower.reachedTarget(10)) {
                    result = limelight.getLatestResult();
                    if (result.isValid() &&movement.milliseconds()>0) {
                        modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                        movement.reset();
                    }
                }
                if (follower.reachedTargetAndStopped(6) && (modules.launcher.launcher.Motorvel() > closecounter || timer. milliseconds()>5000) ){
                    timer.reset();
                    cases = 2;
                    throwtime = -1;
                    modules.intake.setManualPower(1);


                }
                break;
            case 2: //launch pre
                result = limelight.getLatestResult();
                if (result.isValid() &&movement.milliseconds()>0) {
                    modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                    movement.reset();

                }
                modules.intake.setState(Intake.State.MANUAL);

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 125 && throwtime == 0) {
                    modules.launcher.launcher.setAuto(rpmclose2);
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }

                if (timer.milliseconds() > 170 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 1) {

                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 150 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 170 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 3) {
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime = -2;
                    timer.reset();
                    cases=3;
                }
                break;
            case 3: //collect first

                modules.launcher.launcher.setAuto(0);
                modules.intake.setManualPower(1);
                modules.launcher.blocator.setState(Blocator.State.CLOSE);

                follower.setTarget(new Pose(195, -341, Math.toRadians(-90)));
                follower.overallmulti = 0.42;
                if (follower.reachedTarget(5)|| timer.milliseconds()>2500) {
                    modules.launcher.turret.setAuto((angl - 5.5) * TICKSDEG);
                    cases = -3;
                    follower.overallmulti = 1;

                    modules.launcher.launcher.setAuto(rpmclose);
                    follower.setTarget(new Pose(180, -350, Math.toRadians(20)));

                    modules.intake.setManualPower(0);
                    movement.reset();
                }
                break;
            case -3:
                if ((follower.reachedHeading(Math.toRadians(5)) && follower.reachedHeading(4)) || movement.milliseconds() > 700) {
                    cases = 4;
                    timer.reset();
                    follower.setTarget(new Pose(189, -245, Math.toRadians(-90)));
                }
                break;
            case 4:
                if(follower.reachedTarget(10)) {
                    result = limelight.getLatestResult();
                    if (result.isValid() &&movement.milliseconds()>0) {
                        modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                        movement.reset();

                    }

                }
                if (follower.reachedTargetAndStopped(6) && (modules.launcher.launcher.Motorvel() > closecounter || timer. milliseconds()>3500)) {
                    cases = 5;
                    modules.intake.setManualPower(1);
                    throwtime = -1;
                    timer.reset();
                }
                break;
            case 5: //throw first
                result = limelight.getLatestResult();
                if (result.isValid() &&movement.milliseconds()>0) {
                    modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                    movement.reset();

                }
                modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx() * TICKSDEG);

                modules.intake.setState(Intake.State.MANUAL);

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 125 && throwtime == 0) {
                    modules.launcher.launcher.setAuto(rpmclose2);
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }

                if (timer.milliseconds() > 170 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 1) {

                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 150 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 170 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 3) {
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime = -2;
                    timer.reset();
                    cases=6;
                }
                break;
            case 6: //gotosec

                modules.launcher.launcher.setAuto(0);
                follower.setTarget(new Pose(130, -260, Math.toRadians(-90)));
                if (follower.reachedTarget(8)) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    follower.setTarget(new Pose(130, -354, Math.toRadians(-88)));
                    follower.overallmulti = 0.35;
                    cases = 7;
                    timer.reset();
                }
                break;
            case 7: //colect sec
                if (follower.reachedTarget(5.5) ||timer.milliseconds()>2000) {
                    modules.launcher.turret.setAuto((angl ) * TICKSDEG);
                    modules.intake.setManualPower(0.6);
                    follower.overallmulti = 1;

                    modules.launcher.launcher.setAuto(rpmclose + 0.0);
                    follower.setTarget(new Pose(210, -250, Math.toRadians(-90)));


                    cases = 8;
                    timer.reset();
                }
                break;
            case 8:
                if(follower.reachedTarget(10)) {
                    result = limelight.getLatestResult();
                    if (result.isValid() &&movement.milliseconds()>0) {
                        modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                        movement.reset();

                    }

                }
                if (follower.reachedTargetAndStopped(6)&& (modules.launcher.launcher.Motorvel() > closecounter || timer. milliseconds()>3500)) {
                    modules.intake.setManualPower(1);
                    cases = 9;
                    throwtime = -1;
                    timer.reset();
                }
                break;
            case 9: //launchsec
                result = limelight.getLatestResult();
                if (result.isValid() &&movement.milliseconds()>0) {
                    modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                    movement.reset();

                }
                modules.intake.setState(Intake.State.MANUAL);


                modules.intake.setState(Intake.State.MANUAL);

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 125 && throwtime == 0) {
                    modules.launcher.launcher.setAuto(rpmclose2);
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }

                if (timer.milliseconds() > 170 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 1) {

                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 150 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 170 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 3) {
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime = -2;
                    timer.reset();
                    cases=10;
                }
                break;
            case 10: //gotothird
                modules.launcher.launcher.setAuto(0);
                follower.setTarget(new Pose(70, -280, Math.toRadians(-90)));
                if (follower.reachedTarget(8)) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    follower.setTarget(new Pose(70, -362, Math.toRadians(-90)));
                    follower.overallmulti = 0.4;
                    cases = 11;
                    timer.reset();
                }
                break;
            case 11: //colect third
                if (follower.reachedTarget(5.5) || timer.milliseconds()>2000) {

                    modules.intake.setManualPower(0);
                    follower.overallmulti = 1;
                    modules.launcher.turret.setAuto((angl - 4) * TICKSDEG);
                    modules.launcher.launcher.setAuto(rpmclose + 0.0);
                    follower.setTarget(new Pose(225, -225, Math.toRadians(-90)));

                    timer.reset();

                    cases = 12;
                }
                break;
            case 12:
                if(follower.reachedTarget(10)) {
                    result = limelight.getLatestResult();
                    if (result.isValid() &&movement.milliseconds()>0) {
                        modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                        movement.reset();

                    }

                }
                if (follower.reachedTargetAndStopped(6)&& (modules.launcher.launcher.Motorvel() > closecounter || timer. milliseconds()>6000)) {
                    cases = 13;
                    throwtime = -1;
                    timer.reset();
                    modules.intake.setManualPower(1);
                }
                break;

            case 13: //launchthird
                result = limelight.getLatestResult();
                if (result.isValid() &&movement.milliseconds()>0) {
                    modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx()*0.35 * TICKSDEG);
                    movement.reset();

                }

                modules.intake.setState(Intake.State.MANUAL);

                if (throwtime == -1) {
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 125 && throwtime == 0) {
                    modules.launcher.launcher.setAuto(rpmclose2);
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }

                if (timer.milliseconds() > 170 && throwtime == 1) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 1) {

                    throwtime++;
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    timer.reset();

                }
                if (timer.milliseconds() > 150 && throwtime == 2) {
                    modules.launcher.blocator.setState(Blocator.State.CLOSE);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 170 && throwtime == 3) {
                    modules.intake.setManualPower(0);
                }
                if ((timer.milliseconds() > 2000 || (modules.launcher.launcher.Motorvel() > closecounter && timer.milliseconds() > 200)) && throwtime == 3) {
                    modules.intake.setManualPower(1);
                    modules.launcher.blocator.setState(Blocator.State.OPEN);
                    throwtime++;
                    timer.reset();

                }
                if (timer.milliseconds() > 300 && throwtime == 4) {
                    throwtime = -2;
                    timer.reset();
                    cases=14;
                }
                break;
            case 14:
                modules.launcher.launcher.setAuto(0);
                modules.intake.setManualPower(0);
//                follower.setTarget(new Pose(150, -300, Math.toRadians(-90)));
                cases = 15;
                break;

            default:
                telemetry.addData("t", modules.launcher.turret.getCurrentPosition() - result.getTx() * TICKSDEG);
                break;
        }

        Position.setPose(new Pose2d(follower.odo.getPosX(DistanceUnit.CM), follower.odo.getPosY(DistanceUnit.CM), follower.odo.getHeading(AngleUnit.RADIANS)));
        telemetry.addData("h", follower.odo.getPosition().getHeading(AngleUnit.RADIANS));
        telemetry.addData("h", follower.odo.getPosition().getX(DistanceUnit.CM));
        telemetry.addData("h", follower.odo.getPosition().getY(DistanceUnit.CM));
        telemetry.addData("h", follower.localization);
        telemetry.addData("t", throwtime);


        telemetry.addData("heading", follower.getTarget());
        telemetry.update();


    }

    @Override
    public void stop() {
    }

}
