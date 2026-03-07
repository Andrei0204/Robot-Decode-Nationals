package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;

@Config
public class LTur implements IRobotModule, IStateBasedModule {
    public static double target = 0; //cm
    public static double startpos = 0;
    public static double auto = 0.3;
    private static boolean ENABLED = true;

    public DcMotor turret = null;
    public Servo T1;
    public Servo T2;
    public static final double TICKS435 = 8192;    //test
    public static final double TURTICKS = TICKS435 * 4.5;    //test
    public static double TICKSDEG = TURTICKS / 360;

    public State state;
    public static double Kp = 0.0085;           // Proportional gain
    public static double Kd = 0.0008;          // Derivative gain
    public static double Ki = 0.0001;          // Derivative gain
    public static double Kf = 0;// Derivative gain
    public static double Kpc = 0.01;           // Proportional gain
    public static double Kdc = 0.0009;          // Derivative gain
    public static double Kic = 0.000;          // Derivative gain
    public static double Kfc = 0.02;          // Derivative gain
    public static double swap = 4;          // Derivative gain


    public PIDFController controller = new PIDFController(new PIDFCoefficients(Kp, Ki, Kd, Kf));

    public static double profileMaxVelocity = 15, profileAcceleration = 12, profileDeceleration = 10;

    public LTur(HardwareMap hardwareMap, State initialState) {
        if (!ENABLED) {


            T1 = null;
            T2 = null;
        } else {


            T1 = hardwareMap.get(Servo.class, "T1");
            T2 = hardwareMap.get(Servo.class, "T2");
//            T1 = new CoolServo(hardwareMap.get(ServoImplEx.class, "T1"),true,profileMaxVelocity,profileAcceleration,profileDeceleration, initialState.position);
//            T2 = new CoolServo(hardwareMap.get(ServoImplEx.class, "T2"),true,profileMaxVelocity,profileAcceleration,profileDeceleration, initialState.position);

            turret = hardwareMap.get(DcMotor.class, "INTAKE");
            turret.setDirection(DcMotorSimple.Direction.REVERSE);
            T2.setDirection(Servo.Direction.REVERSE);
            T1.setDirection(Servo.Direction.REVERSE);

        }
        this.state = initialState;
        setangle(0);
        setState(State.AUTO);

    }


    public double getTarget() {
        return target;
    }

    public void setTarget(double newTarget) {
        this.target = newTarget;
    }


    public double getCurrentPosition() {
        return T1.getPosition();
    }

    public double getCurrentPositionenc() {
        return turret.getCurrentPosition() / TICKSDEG;
    }


    public State getState() {
        return state;
    }

    public void setState(State newState) {
        if (state == newState) return;
        this.state = newState;
    }

    public void setangle(double angle) {
        if (angle == angle) {
            auto = angle;
            if (auto < -90)
                auto = -90;
            if (auto > 180)
                auto = 180;
        }

    }

    public double getAuto() {
        return auto;
    }

    @Override
    public void update() {

        if (!ENABLED) return;
        updateState();
        updateHardware();
//        T1.update();
//        T2.update();
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


        double currentAngle = turret.getCurrentPosition() / TICKSDEG; // in ticks
        if (Math.abs(currentAngle - auto) > swap) {
            controller.setP(Kp);
            controller.setI(Ki);
            controller.setF(Kf);
            controller.updateFeedForwardInput(0);
            controller.setD(Kd);
        } else {
            controller.setP(Kpc);
            controller.setI(Kic);
            controller.setF(Kfc);
            controller.setD(Kdc);
            controller.updateFeedForwardInput(Math.signum(auto - currentAngle));
        }


        controller.setTargetPosition(auto);
        controller.updatePosition(currentAngle);
        // ----- PD CONTROL -----
        double power = controller.run();
        // ----- CLAMP POWER -----
        power = Math.max(-1.0, Math.min(1.0, power));
        // ----- SEND TO SERVO -----
        // Continuous servo expects 0-1 scale where 0.5 = stop
        double servoCommand = 0.5 + power / 2.0;
        T1.setPosition(servoCommand);
        T2.setPosition(servoCommand);
//
//        T1.setPosition(target+offsetpos);
//        T2.setPosition(target);
    }

    public enum State {
        START(startpos),
        AUTO(auto);
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

}

