package org.firstinspires.ftc.teamcode.ROBOT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.CoolServo;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;
@Config
public class LTur implements IRobotModule, IStateBasedModule {
    public static double target = 0; //cm
    public static double startpos = 0;
    public static double auto = 0.3;
    public static   double angle = 104;
    public static   double ticks = 0.00297;
    private static boolean ENABLED = true;

    public DcMotor turret = null;



    public Servo T1  ;
    public Servo    T2;
    public static final double TICKS435 = 8192 ;    //test
    public static final double TURTICKS = TICKS435 *4.5;    //test
    public static double TICKSDEG = TURTICKS/360;

    public LTur.State state;


    public static double profileMaxVelocity = 15, profileAcceleration = 12, profileDeceleration = 10;

    public LTur(HardwareMap hardwareMap, LTur.State initialState) {
        if (!ENABLED) {


            T1 = null;
            T2 = null;
        } else {


            T1 = hardwareMap.get(Servo.class, "T1");
            T2 = hardwareMap.get(Servo.class, "T2");
//            T1 = new CoolServo(hardwareMap.get(ServoImplEx.class, "T1"),true,profileMaxVelocity,profileAcceleration,profileDeceleration, initialState.position);
//            T2 = new CoolServo(hardwareMap.get(ServoImplEx.class, "T2"),true,profileMaxVelocity,profileAcceleration,profileDeceleration, initialState.position);

            turret =  hardwareMap.get(DcMotor.class, "INTAKE");
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
        return turret.getCurrentPosition()/TICKSDEG;
    }


    public LTur.State getState() {
        return state;
    }

    public void setState(LTur.State newState) {
        if (state == newState) return;
        this.state = newState;
    }

    public void setangle(double angle)
    {
        angle =angle+this.angle;
        double upmax=0.85;
        double downmax=0;
        double tickstodegree=ticks  ;
        this.auto  =angle*tickstodegree ;
        if(auto<downmax)
            auto=downmax;
        if(auto>upmax)
            auto=upmax;
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
        T1.setPosition(target);
        T2.setPosition(target);
    }

    public enum State {
        START(startpos),
        AUTO(auto);
        public final LTur.State nextState;
        public double position;

        State(double position) {
            this.position = position;
            this.nextState = this;
        }

        State(double position, LTur.State nextState) {
            this.position = position;
            this.nextState = nextState;
        }
    }

}
