package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

//public static  double m = 10.781;
//public static double n = 93.65;
//public void LaunchtoRpm(double velocity) {
//    setAuto(((m * velocity + n) / 60 )/ 100
//    );
//}
//public double getLaunchtoRpm(double velocity)
//{
//    return ((m * velocity + n) / 60 )/ 100;
//}
//public static double kP = 0.2, kI = 0, kD = 0, kf = 0; //TODO Valori PID!!!!
//
@Config
@Configurable

public class Launcher implements IStateBasedModule, IRobotModule {

    public static double m = 10.8;
    public static double n = 93;
    private static final Logger log = LoggerFactory.getLogger(Launcher.class);
    public static double TICKS_PER_REV = 28;

    public static double startpos = 0;
    public static final double finalpos = 50;
    public static double autopose = 0;
    public static double target = 0; //cm
    public static double currentPosition = startpos;
    public static double kP = 0.7, kI = 0, kD = 0, kf = 0; //TODO Valori PID!!!!
    public static double kS = 0, kV = 2.9, kA = 0, counter = 0.025;
    public static double kv2 = 0.9, kp2 = 0.6;
    public static double kvd = -0.2, kpd = -0.1;

    private static boolean ENABLED = true;


    public double manualPower = 0;
    public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

    public State state;
    public EXTRUNMODE runMode;
    VoltageSensor voltage;

    public Motor launcher;

    public DcMotor motorOne;
    public Motor launcher2;
    public DcMotor motorTwo;

    double actualKs = 0;

    public Launcher(HardwareMap hardwareMap, State initialState) {
        Motor encoder;
        if (!ENABLED) {

            encoder = null;
            motorOne = null;
        } else {

            encoder = new Motor(hardwareMap, "INTAKE", Motor.GoBILDA.BARE);
            launcher = new Motor(hardwareMap, "L1", Motor.GoBILDA.BARE);
            launcher2 = new Motor(hardwareMap, "L2", Motor.GoBILDA.BARE);
            launcher.encoder = launcher2.encoder;

            motorOne = launcher.motor;
            motorTwo = launcher2.motor;
            launcher.setInverted(false);
            launcher2.setInverted(true);
            launcher.setRunMode(Motor.RunMode.VelocityControl);
            launcher2.setRunMode(Motor.RunMode.VelocityControl);
            voltage = hardwareMap.voltageSensor.iterator().next();

        }
        this.state = initialState;
    }

    public void LaunchtoRpm(double velocity) {
        if (velocity == velocity)
            setAuto(((m * velocity + n) / 60) / 100
            );
        else
            setAuto(0.4);
    }

    public double getLaunchtoRpm(double velocity) {
        return ((m * velocity + n) / 60) / 100;
    }

    public double getSpeed() {
        return ((Motorvel() * 60 - n) / m);
    }

    public double getAmps1() {
        return ((DcMotorEx) motorOne).getCurrent(CurrentUnit.AMPS);
    }

    public double getpow1() {
        return (motorOne).getPower();
    }

    public double getpow2() {
        return (motorTwo).getPower();
    }

    public double getAmps2() {
        return ((DcMotorEx) motorTwo).getCurrent(CurrentUnit.AMPS);
    }

    public void setAuto(double auto) {
        autopose = -auto;
    }

    public static double getAutopose() {
        return autopose;
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
        return motorTwo.getCurrentPosition();
    }

    public double getRealCurrentPosition() {
        return motorTwo.getCurrentPosition() / TICKS_PER_REV;
    }

    public boolean atTarget() {
        return Math.abs(Motorvel() - target / TICKS_PER_REV) < 150;
    }

    public double Revolutions(double ticks) {
        return ticks / TICKS_PER_REV;
    }

    public double Motorvel() {
        return -Revolutions(launcher2.encoder.getRawVelocity());
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
        return (Math.abs(target - getCurrentPosition())) / TICKS_PER_REV <= tolerance;
    }

    public void updateStateValues() {
        State.MANUAL.position = currentPosition;
        setTarget(currentPosition * TICKS_PER_REV);
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
            target = currentPosition * TICKS_PER_REV;
        } else {
            runMode = EXTRUNMODE.PID;
            switch (state) {

                case FULL:
                    setTarget(finalpos);
                    break;
                case START:
                    setTarget(startpos);
                    break;
                case TEST:
                    setTarget(autopose);
                    break;

            }
        }
    }

    @Override
    public void updateHardware() {


        switch (runMode) {


            case PID:
                if (-Motorvel() / 100 > target + counter) {
                    launcher.setVeloCoefficients(kP, kI, kD);
                    launcher2.setVeloCoefficients(kP, kI, kD);

                    launcher.setFeedforwardCoefficients(kS + actualKs, kV, kA);
                    launcher2.setFeedforwardCoefficients(kS + actualKs, kV, kA);
                } else if (
                        -Motorvel() / 100 < target - counter
                ) {
                    launcher.setVeloCoefficients(kpd, kI, kD);
                    launcher2.setVeloCoefficients(kpd, kI, kD);

                    launcher.setFeedforwardCoefficients(kS + actualKs, kvd, kA);
                    launcher2.setFeedforwardCoefficients(kS + actualKs, kvd, kA);
                } else {
                    launcher.setVeloCoefficients(kp2, kI, kD);
                    launcher2.setVeloCoefficients(kp2, kI, kD);

                    launcher.setFeedforwardCoefficients(kS + actualKs, kv2, kA);
                    launcher2.setFeedforwardCoefficients(kS + actualKs, kv2, kA);
                }
                launcher.set(target);
                launcher2.set(target);


                break;
            case MANUAL: {

                break;
            }
        }


        currentPosition = Motorvel() / TICKS_PER_REV;

    }

    public enum State {
        MANUAL(currentPosition),
        START(startpos),
        TEST(autopose),
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

    public enum EXTRUNMODE {
        PID, MANUAL
    }
}
