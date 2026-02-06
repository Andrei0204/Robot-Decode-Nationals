package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;

public class Pivot implements IRobotModule, IStateBasedModule {
    public static double target = 0; //cm
    public static double startpos = 0.4;
    public static double auto = 0.5;

    private static boolean ENABLED = true;
    public Servo pivot = null;



    public Pivot.State state;



    public Pivot(HardwareMap hardwareMap, Pivot.State initialState) {
        if (!ENABLED) {


            pivot = null;
        } else {

            pivot = hardwareMap.get(Servo.class, "PIVOT");

        }
        this.state = initialState;
    }



    public double getTarget() {
        return target;
    }

    public void setTarget(double newTarget) {
        this.target = newTarget;
    }



    public double getCurrentPosition() {
        return pivot.getPosition();
    }


    public Pivot.State getState() {
        return state;
    }

    public void setState(Pivot.State newState) {
        if (state == newState) return;
        this.state = newState;
    }

    public void setangle(double angle)
    {
        double max =1;
        double min=0.1;
        double degmin=25;
        double degmax=58;
        auto =max+(angle-degmin)/(degmax-degmin)*(min-max);
        if(auto<0.08)
            auto=0.08;
        if(auto>1)
            auto=1;
    }
    @Override
    public void update() {
        if (!ENABLED) return;
        updateState();
        updateHardware();

    }

    @Override
    public void updateState() {
        switch (state) {
            case START:
                setTarget(startpos);
                break;
            case AUTO:
                setTarget(auto);
                break;
        }
    }

    @Override
    public void updateHardware() {
        pivot.setPosition(target);
    }

    public enum State {
        START(startpos),
        AUTO(auto);
        public final Pivot.State nextState;
        public double position;

        State(double position) {
            this.position = position;
            this.nextState = this;
        }

        State(double position, Pivot.State nextState) {
            this.position = position;
            this.nextState = nextState;
        }
    }

}
