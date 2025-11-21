package org.firstinspires.ftc.teamcode.ROBOT;
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Position;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RobotControl;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Point;

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
@TeleOp
public class RobotKickblue extends OpMode {
    DcMotor BL = null, BR = null, FR = null, FL = null;
    DcMotor push;
    RoboModules modules;
    RobotControl control;
    double oldTime = 0;
    public   double alpha = 0;
    double deltax = 0;
    double deltay = 0;
    public static double deltahhypot = 0;
    Point redGoal = new Point(136 * 2.54, -140 * 2.54);
    Point blueGoal = new Point(140 * 2.54, -2 * 2.54);
    boolean isred=false;
    GoBildaPinpointDriver odo = null;
    Boolean odobased=true;
    Boolean llbased=false;
    ElapsedTime movement;
    Limelight3A limelight;
    LLResult result=null;
    @Override
    public void init() {

// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        telemetry= new MultipleTelemetry( telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(54, -164.5, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        movement = new ElapsedTime();
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        push = hardwareMap.get(DcMotor.class, "PUSH");

        modules = new RoboModules(hardwareMap);
        control = new RobotControl(gamepad1, gamepad2, modules);
        push.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO modifca de aici daca nu se misca bine robotu

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        modules.launcher.turret.setState(Turret.State.TEST);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();
    }


    @Override
    public void init_loop() {
        if(gamepad1.dpad_up)
            modules.launcher.turret.turret.resetEncoder();
        if(gamepad1.dpad_down)
            odo.setPosition(new Pose2D(DistanceUnit.CM,Position.getPose().getX(),Position.getPose().getY()+20,AngleUnit.RADIANS,0));

    }

    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    public double normalizaion(double angle)
    {
        return (angle +Math.PI) %(2*Math.PI) -Math.PI;
    }


    @Override
    public void loop() {
        odo.update();
        Pose2D pos = odo.getPosition();
        control.update();
        double x =-gamepad1.left_stick_x; // Remember, Y stick is reversed!
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        FL.setPower((y + x + rx)/denominator);
        FR.setPower((y - x - rx)/denominator);
        BL.setPower((y - x + rx)/denominator);
        BR.setPower((y + x - rx)/denominator);


//        push.setPower(-0.01);
        if(gamepad1.dpad_right)
            isred=false;
        if(gamepad1.dpad_up)
            odo.setPosition(new Pose2D( DistanceUnit.CM,154,-154,AngleUnit.RADIANS,odo.getHeading(AngleUnit.RADIANS)));
        if(isred) {
            deltax = +pos.getX(DistanceUnit.CM) - redGoal.x;
            deltay = -pos.getY(DistanceUnit.CM) + redGoal.y;
        }else
        {
            deltax = +pos.getX(DistanceUnit.CM) - blueGoal.x;
            deltay = -pos.getY(DistanceUnit.CM) + blueGoal.y;
        }
        deltahhypot= Math.hypot(deltay,deltax);
        double impartire =deltay/deltax;
        alpha = -Math.atan(impartire);

        double anglelocal =  pos.getHeading(AngleUnit.RADIANS)-alpha ;
        double anglenorm = -Math.toDegrees(normalizaion(anglelocal) );
        if(anglenorm<=-140)
            anglenorm +=360;


        if(odobased)
            modules.launcher.turret.setAuto(anglenorm* TICKSDEG);
        if(!odobased && !llbased)
            modules.launcher.turret.setAuto(0* TICKSDEG);
        if(llbased)
        {
            result = limelight.getLatestResult();
            if (result.isValid() ) {
                if( movement.milliseconds()>200 &&Math.abs(modules.launcher.turret.getCurrentPosition() - result.getTx() * TICKSDEG)/TICKSDEG >1)
                { modules.launcher.turret.setAuto(modules.launcher.turret.getCurrentPosition() - result.getTx() * TICKSDEG);
                movement.reset();
                }
            }
        }
        if(gamepad2.dpad_down) {
            odobased = false;
            llbased=false;
        }
        if(gamepad2.dpad_right){
            odobased = false;
            llbased=true;
        }
        if(gamepad2.dpad_left)
        {
            odobased = true;
            llbased=false;
        }

        if(gamepad2.left_trigger>0.2)
            push.setPower(-gamepad2.left_trigger);
        if(gamepad2.right_trigger>0.2)
            push.setPower(gamepad2.right_trigger);
        telemetry.addData("dx",deltax) ;
        telemetry.addData("dy",deltay);
        telemetry.addData("dh",Math.abs(deltahhypot));
//        telemetry.addData("alpha",alpha);
//        telemetry.addData("anglelocal",anglelocal);
//        telemetry.addData("anglenorm",anglenorm);
//        telemetry.addData("alphagrade",Math.toDegrees(alpha));
        telemetry.addData("turret",modules.launcher.turret.getState());
        telemetry.addData("turret",modules.launcher.turret.getTarget());
//        telemetry.addData("push", push.getPower());
        telemetry.addData("push", modules.launcher.launcher.Motorvel());
        telemetry.addData("AMPS", modules.launcher.launcher.getAmps());
        telemetry.addData("powerintake", modules.intake.intake.getPower());
//        telemetry.addData("push", modules.launcher.pivot.getTarget());
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);


        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();


    }

    @Override
    public void stop() {
    }

}
