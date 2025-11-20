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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Position;

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
public class ShobyAutoRed extends OpMode
{
    Position pose= null;
    DcMotor BL=null,BR=null,FR=null,FL=null;
    DcMotor l1,l2,l3;
    Servo lopata;
    int cases = 0;
    ElapsedTime timp=null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        pose =new Position();
        lopata =  hardwareMap.get(Servo.class, "lopata");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        BR  = hardwareMap.get(DcMotor.class, "BR");
        FR  = hardwareMap.get(DcMotor.class, "FR");
        FL  = hardwareMap.get(DcMotor.class, "FL");
        l1  = hardwareMap.get(DcMotor.class, "l1");
        l2  = hardwareMap.get(DcMotor.class, "l2");
        l3  = hardwareMap.get(DcMotor.class, "l3");
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        timp= new ElapsedTime();

    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        lopata.setPosition(0.42);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        if(cases==0)
        {
            l1.setPower(-0.8 );
            l2.setPower(-0.8);
            l3.setPower(0.8);
            cases=1;
            timp.reset();
        }
        if(cases==1)
        {
            FL.setPower(0.4);
            BL.setPower(0.4);
            FR.setPower(0.4);
            BR.setPower(0.4);
            if(timp.milliseconds()>3000) {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                cases = 2;
                timp.reset();
            }

        }
        if(cases==2){
            FL.setPower(-0.4);
            BL.setPower(-0.4);
            FR.setPower(0.4);
            BR.setPower(0.4);
            if(timp.milliseconds()>350) {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                cases = 3;
                timp.reset();
            }
        }
        if(cases==3) {
            if (timp.milliseconds() > 1000 && timp.milliseconds()<2500) {
                lopata.setPosition(0.6);
            }
            if (timp.milliseconds() > 2500 && timp.milliseconds()<3200) {
                lopata.setPosition(0.42);
            }
            if (timp.milliseconds() > 3200 && timp.milliseconds()<4000) {
                lopata.setPosition(0.6);
            }
            if (timp.milliseconds() > 4000 && timp.milliseconds()<5000) {
                lopata.setPosition(0.42);
            }
            if (timp.milliseconds() > 5000 && timp.milliseconds()<6000) {
                lopata.setPosition(0.6);
            }
            if (timp.milliseconds() > 6000) {
                lopata.setPosition(0.42);
                cases=4;
                timp.reset();
            }

        }
        if(cases==4) {
            FL.setPower(-0.4);
            BL.setPower(0.4);
            FR.setPower(0.4);
            BR.setPower(-0.4);
            if (timp.milliseconds() > 1000)
            {
                FL.setPower(0);
                BL.setPower(0);
                FR.setPower(0);
                BR.setPower(0);
                cases=5;
            }
        }


    }

    @Override
    public void stop() {
    }

}



