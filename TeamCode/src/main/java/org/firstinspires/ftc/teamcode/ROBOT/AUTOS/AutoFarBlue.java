package org.firstinspires.ftc.teamcode.ROBOT.AUTOS;
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
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl.distance;
import static org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl.farcounter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Point;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Pose;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Vector;

import java.util.List;

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
public class AutoFarBlue extends OpMode {
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
    public static double errortur = 0;
    public static double rpm = 0.9;
    public static double angl = 45;
    public static double kppeleme = 0.35;
    public static double time = 0;
    Limelight3A limelight;
    LLResult result = null;
    InterpLUT far = new InterpLUT();

    @Override
    public void init() {
        time = 0;
        far.add(270, 44);
        far.add(300,44 );
        far.add(330, 42);
        far.add(360, 41);
        far.createLUT();
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

        limelight.pipelineSwitch(1);
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
        odo.setPosition(new Pose2D(DistanceUnit.CM, -15, -128, AngleUnit.RADIANS, 0));
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */

    public void LL() {
        result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> detectorResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult dr : detectorResults) {
                if (dr.getFiducialId() == 20) {
                    Vector distances = new Vector(Math.abs(dr.getRobotPoseTargetSpace().getPosition().x), dr.getRobotPoseTargetSpace().getPosition().z);
                    errortur = Math.abs(dr.getTargetXDegrees());
                    distance = distances.getMagnitude() * 100;
                    if (movement.milliseconds() > 0 && Math.abs(modules.launcher.turret.getCurrentPosition() - dr.getTargetXDegrees() * TICKSDEG) / TICKSDEG > 1
                    ) {
                        double heading_error = dr.getTargetXDegrees();
                        double goodpower = heading_error * kppeleme;
                        modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - goodpower * TICKSDEG);
                        movement.reset();
                    }
                }
            }
        }
    }

    @Override
    public void loop() {
        modules.update();
        follower.update();
        switch (cases) {
            case 1: //Gotoprelaunch
                LL();
                modules.launcher.pivot.setState(Pivot.State.AUTO);
                modules.launcher.launcher.setState(Launcher.State.TEST);
                modules.launcher.turret.setState(Turret.State.TEST);
                modules.launcher.launcher.setAuto(rpm);
                modules.intake.setState(Intake.State.MANUAL);
                if (!follower.reachedTarget(6))
                    timer.reset();
                follower.setTarget(new Pose(3, -111, Math.toRadians(18)));


                if (follower.reachedTargetAndStopped(6) && ((modules.launcher.launcher.Motorvel() > farcounter && errortur < 4) || timer.milliseconds() > 4000)) {
                    timer.reset();
                    cases = 2;
                    throwtime = -1;
                    modules.intake.setManualPower(1);


                }
                break;
            case 2: //launch pre
                LL();
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
                    if (timer.milliseconds() > 125 && throwtime == 2) {
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
                        cases = 3;
                        timer.reset();
                    }
                }


                break;
            case 3: //collect first
                modules.launcher.turret.setAuto(0);
                follower.overallmulti = 0.85;
                follower.setTarget(new Pose(20, -10, Math.toRadians(126)));

                modules.intake.setManualPower(1);
                modules.launcher.launcher.setAuto(0);
                modules.launcher.blocator.setState(Blocator.State.CLOSE);
                if (follower.reachedTarget(4) || timer.milliseconds() > 2000) {
                    timer.reset();
                    cases = 4;
                }
                break;
            case 4:
                follower.overallmulti = 1;
                follower.setTarget(new Pose(-10, -14, Math.toRadians(126)));


                if (follower.reachedTarget(4) || timer.milliseconds() > 3000) {
                    cases = 5;
                    modules.intake.setManualPower(0);
                    follower.setTarget(new Pose(3, -111, Math.toRadians(18)));

                    modules.launcher.launcher.setAuto(rpm);
                }
                break;
            case 5:

                modules.launcher.pivot.setState(Pivot.State.AUTO);
                modules.launcher.launcher.setState(Launcher.State.TEST);
                modules.launcher.turret.setState(Turret.State.TEST);

                modules.intake.setState(Intake.State.MANUAL);
                if (!follower.reachedTarget(6))
                    timer.reset();
                if (!follower.reachedHeading(Math.toRadians(10)))
                    modules.launcher.turret.setAuto(0);
                else
                    LL();

                if (follower.reachedTargetAndStopped(6) && follower.reachedHeading(Math.toRadians(7)) && ((modules.launcher.launcher.Motorvel() > farcounter && errortur < 4) || timer.milliseconds() > 4000)) {
                    timer.reset();
                    cases = 6;
                    throwtime = -1;
                    modules.intake.setManualPower(1);

                }
                break;
            case 6:
                modules.launcher.turret.setAuto(0);
                LL();
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
                    if (timer.milliseconds() > 125 && throwtime == 2) {
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
                        cases = 12;
                    }

                }
                break;
            case 7:
                follower.setTarget(new Pose(3, -50, Math.toRadians(25)));

                modules.intake.setManualPower(0);
                modules.launcher.launcher.setAuto(0);
                break;
            case 8: //collect first
                follower.overallmulti = 0.85;
                modules.launcher.turret.setAuto(0);
                follower.setTarget(new Pose(60, -85, Math.toRadians(90)));
                modules.intake.setManualPower(1);
                modules.launcher.launcher.setAuto(0);
                modules.launcher.blocator.setState(Blocator.State.CLOSE);
                if (follower.reachedTarget(4)) {
                    timer.reset();
                    cases = 9;
                }
                break;
            case 9:
                follower.overallmulti = 0.5;
                follower.setTarget(new Pose(60, -20, Math.toRadians(90)));
                if (follower.reachedTarget(4) || timer.milliseconds() > 2000) {
                    follower.overallmulti = 1;
                    cases = 10;
                    modules.intake.setManualPower(0);
                    follower.setTarget(new Pose(3, -111, Math.toRadians(18)));
                    modules.launcher.launcher.setAuto(rpm);
                }
                break;
            case 10:

                modules.launcher.pivot.setState(Pivot.State.AUTO);
                modules.launcher.launcher.setState(Launcher.State.TEST);
                modules.launcher.turret.setState(Turret.State.TEST);

                modules.intake.setState(Intake.State.MANUAL);
                if (!follower.reachedTarget(6))
                    timer.reset();
                if (!follower.reachedHeading(Math.toRadians(30)))
                    modules.launcher.turret.setAuto(0);
                else
                    LL();

                if (follower.reachedTargetAndStopped(6) && follower.reachedHeading(Math.toRadians(7)) && ((modules.launcher.launcher.Motorvel() > farcounter && errortur < 4) || timer.milliseconds() > 4000)) {
                    timer.reset();
                    cases = 11;
                    throwtime = -1;
                    modules.intake.setManualPower(1);

                }
                break;
            case 11:
                modules.launcher.turret.setAuto(0);
                LL();
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
                    if (timer.milliseconds() > 125 && throwtime == 2) {
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
                        cases = 12;
                    }

                }
                break;
            case 12:
                follower.overallmulti = 0.6;
                modules.launcher.turret.setAuto(0);

                if (time == 0)
                    follower.setTarget(new Pose(-10, -14, Math.toRadians(100)));
                else
                    follower.setTarget(new Pose(-13, -14, Math.toRadians(100)));
                modules.intake.setManualPower(1);
                modules.launcher.launcher.setAuto(0);
                modules.launcher.blocator.setState(Blocator.State.CLOSE);
                if (timer.milliseconds() > 3000) {
                    timer.reset();
                    cases = 13;
                    modules.intake.setManualPower(0);
                    follower.overallmulti = 0.85;
                    follower.setTarget(new Pose(3, -111, Math.toRadians(18)));
                    modules.launcher.launcher.setAuto(rpm);
                }
                break;
            case 13:

                modules.launcher.pivot.setState(Pivot.State.AUTO);
                modules.launcher.launcher.setState(Launcher.State.TEST);
                modules.launcher.turret.setState(Turret.State.TEST);

                modules.intake.setState(Intake.State.MANUAL);
                if (!follower.reachedTarget(6))
                    timer.reset();
                if (!follower.reachedHeading(Math.toRadians(30)))
                    modules.launcher.turret.setAuto(0);
                else
                    LL();

                if (follower.reachedTargetAndStopped(6) && follower.reachedHeading(Math.toRadians(7)) && ((modules.launcher.launcher.Motorvel() > farcounter && errortur < 4) || timer.milliseconds() > 4000)) {
                    timer.reset();
                    cases = 14;
                    throwtime = -1;
                    modules.intake.setManualPower(1);

                }
                break;
            case 14:
                modules.launcher.turret.setAuto(0);
                LL();
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
                    if (timer.milliseconds() > 125 && throwtime == 2) {
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
                        cases = 7;
                    }

                }
                break;
        }

        if (distance > 270 && distance < 360)
            modules.launcher.pivot.setangle(far.get(distance));
        else
            modules.launcher.pivot.setangle(angl);
        if (Launcher.getAutopose() != 0) {
            if (modules.launcher.launcher.Motorvel() > farcounter) {
                modules.launcher.launcher.setAuto(farcounter / 100);
            } else if (throwtime <= 0)
                modules.launcher.launcher.setAuto(rpm);
            else
                modules.launcher.launcher.setAuto(rpm);
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
