package org.firstinspires.ftc.teamcode.ROBOT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.CoolMotor;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;

@Config
public class Spinner implements IStateBasedModule, IRobotModule {


    public static final double TICKS435 = 103.8;    //test
    public static final double TURTICKS = TICKS435 * 3;    //test
    public static double TICKSDEG = TURTICKS / 360;

    public static double startpos = 0 * TICKSDEG;
    public static final double finalpos = 9 * TICKSDEG;
    public static double autopose = 0;
    public static double sec = 100;
    public static double behsec = -176;
    public static double behthird = -270;
    public static double third = 200;
    public static double launch = 340;
    public static double jiggf = 5;
    public static double jiggb = -5;
    public static double target = 0; //cm
    public static double currentPosition = startpos;
    public static double kP = 0.01, kI = 0, kD = 0.0005; //TODO Valori PID!!!!
    public static double f = 0;

    private static boolean ENABLED = true;
    public CoolMotor spinner = null;


    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

    public State state;
    public EXTRUNMODE runMode;


    public Spinner(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {


            spinner = null;
        } else {

            spinner = new CoolMotor(hardwareMap.get(DcMotorEx.class, "SPINNER"), CoolMotor.RunMode.RUN, false);


        }
        this.state = initialState;
    }


    public void setManualPower(double power) {
        this.manualPower = power;
    }

    public double getTarget() {
        return target;
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

    public double getCurrentPosition() {
        return spinner.getCurrentPosition();
    }

    public double getRealCurrentPosition() {
        return spinner.getCurrentPosition() / TICKSDEG;
    }

    public boolean atTarget() {
        return Math.abs(getRealCurrentPosition() - target / TICKSDEG) < 5;
    }

    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (state == newState) return;
        this.state = newState;
    }

    public boolean reachedPosition(double tolerance) {
        if (state == State.MANUAL)
            return false;
        return (Math.abs(target - getCurrentPosition())) / TICKSDEG <= tolerance;
    }

    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(currentPosition * TICKSDEG);
    }

    public void setAuto(double auto) {
        autopose = auto;
    }

    public double getAuto() {
        return autopose;
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
            target = currentPosition * TICKSDEG;
        } else {
            runMode = EXTRUNMODE.PID;
            switch (state) {

                case FULL:
                    setTarget(finalpos);
                    break;
                case START:
                    setTarget(startpos);
                    break;
                case SEC:

                    setTarget(sec);
                    break;
                case THIRD:
                    setTarget(third);
                    break;
                case BEHSEC:
                    setTarget(behsec);
                    break;
                case BEHTHIRD:
                    setTarget(behthird);
                    break;
                case TEST:
                    setTarget(autopose);
                    break;
                case JIGF:
                    setTarget(jiggf);
                    break;
                case JIGB:
                    setTarget(jiggf);
                    break;
                case Launch:
                        setTarget(launch);
                    break;

            }
        }
    }

    @Override
    public void updateHardware() {
        pid = new PIDCoefficients(kP, kI, kD);

        switch (runMode) {
            case PID:
                spinner.setMode(CoolMotor.RunMode.PID);


                spinner.setPIDF(pid, f);


                spinner.calculatePowerFF(getRealCurrentPosition(), target);
                break;
            case MANUAL: {

                spinner.setMode(CoolMotor.RunMode.RUN);

                spinner.setPower(manualPower);
                break;
            }
        }


        currentPosition = spinner.getCurrentPosition() / TICKSDEG;
        spinner.update();

    }

    public enum State {
        MANUAL(currentPosition),
        START(startpos),
        TEST(autopose),
        SEC(sec),
        THIRD(third),
        JIGB(jiggb),
        JIGF(jiggf),
        BEHSEC(behsec),
        BEHTHIRD(behthird),
        Launch(launch),
        FULL(finalpos);
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

    public void counter(int k) {
        setRunMode(Spinner.EXTRUNMODE.PID);
        switch (k) {
            case 0:
                setState(State.START);
                break;
            case 1:
                setState(State.SEC);
                break;
            case 2:
                setState(State.THIRD);
                break;
            case -1:
                setState(State.BEHSEC);
                break;
            case -2:
                setState(State.BEHTHIRD);
                break;


        }

    }

    ;


    public enum EXTRUNMODE {
        PID, MANUAL
    }
}

