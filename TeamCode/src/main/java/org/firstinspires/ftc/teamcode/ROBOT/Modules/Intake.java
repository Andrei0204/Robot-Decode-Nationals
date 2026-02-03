package org.firstinspires.ftc.teamcode.ROBOT.Modules;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;

@Config
public class Intake implements IStateBasedModule, IRobotModule {
    public static double currentPosition = 0;


    private static boolean ENABLED = true;
    public CoolMotor intake = null;


    public double manualPower = 0;
    public double target = 0;


    public State state;
    public EXTRUNMODE runMode;



    public Intake(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {


            intake = null;
        } else {

            intake = new CoolMotor(hardwareMap.get(DcMotorEx.class, "INTAKE"), CoolMotor.RunMode.RUN,false);
            runMode= EXTRUNMODE.MANUAL;
            intake.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.FLOAT);
            setState(State.MANUAL);
        }
        this.state = initialState;
    }


    public void setManualPower(double power) {
        this.manualPower = power;
    }
    public double getManualPower() {
        return manualPower;
    }



    public void setTarget(double newTarget) {
        this.target = newTarget;
    }

    public EXTRUNMODE getRunMode() {
        return runMode;
    }

    public void setRunMode(EXTRUNMODE runMode) {
        this.runMode = runMode;
    }




    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (state == newState) return;
        this.state = newState;
    }



    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(0);
    }


    @Override
    public void update() {
        if (!ENABLED) return;
        updateStateValues();
        updateState();
        updateHardware();

    }

    @Override
    public void updateState() {
        if (!ENABLED) return;
        if (Math.abs(manualPower) >= 0.1) {
            runMode = EXTRUNMODE.MANUAL;
            setState(State.MANUAL);
        }
    }

    @Override
    public void updateHardware() {
        switch (runMode) {
            case MANUAL: {

                intake.setMode(CoolMotor.RunMode.RUN);

                intake.setPower(manualPower);
                break;
            }
        }
        intake.update();

    }

    public enum State {
        MANUAL(currentPosition);

        public final State nextState;
        public double position;

        State(double position) {
            this.position = position;
            this.nextState = this;
        }

        State(double position, State nextState) {
            this.position = position;
            this.nextState = nextState;
        }
    }

    public enum EXTRUNMODE {
        MANUAL
    }
}
