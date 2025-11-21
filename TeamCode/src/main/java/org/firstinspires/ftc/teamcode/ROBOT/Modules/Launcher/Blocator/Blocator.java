package org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Blocator;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;
@Config
public class Blocator implements IRobotModule, IStateBasedModule {
    public static double target = 0; //cm
    public static double openpos = 0.07;
   public static double closepos = 0.2 ;

    private static boolean ENABLED = true;
    public Servo blocator = null;



    public Blocator.State state;



    public Blocator(HardwareMap hardwareMap, Blocator.State initialState) {
        if (!ENABLED) {


            blocator = null;
        } else {

            blocator = hardwareMap.get(Servo.class, "blocator ");

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
        return blocator.getPosition();
    }


    public Blocator.State getState() {
        return state;
    }

    public void setState(Blocator.State newState) {
        if (state == newState) return;
        this.state = newState;
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
                case OPEN:
                    setTarget(openpos);
                    break;
                case CLOSE:
                    setTarget(closepos);
                    break;
            }
    }

    @Override
    public void updateHardware() {
        blocator.setPosition(target);
    }

    public enum State {
        OPEN(openpos),
        CLOSE(closepos);
        public final Blocator.State nextState;
        public double position;

        State(double position) {
            this.position = position;
            this.nextState = this;
        }

        State(double position, Blocator.State nextState) {
            this.position = position;
            this.nextState = nextState;
        }
    }

}
