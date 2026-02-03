package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.gamepads.StickyGamepad;

public class RobotControl implements IRobotModule {
    Gamepad gamepad1, gamepad2;
    RoboModules modules = null;
    ElapsedTime backtime;
    ElapsedTime opentime;
    ElapsedTime timer;

    public RobotControl(Gamepad gamepad1, Gamepad gamepad2, RoboModules modules) {

        this.gamepad1 = gamepad1;
        backtime = new ElapsedTime();
        opentime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        timer = new ElapsedTime();
        this.modules = modules;
    }

    private void UpIn(){

        if(gamepad1.leftBumperWasPressed())
            modules.intake.setManualPower(0);
        if(gamepad1.rightBumperWasPressed())
            modules.intake.setManualPower(1);
        if(gamepad1.dpadUpWasPressed())
            modules.intake.setManualPower(-1);
    }
    @Override
    public void update() {
        UpIn();
    }
}
