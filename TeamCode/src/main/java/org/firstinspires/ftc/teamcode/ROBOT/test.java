package org.firstinspires.ftc.teamcode.ROBOT;

import static org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret.TICKSDEG;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Blocator.Blocator;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Launcher.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.RoboModules;

@TeleOp
@Config
public class test extends LinearOpMode {
    RoboModules modules;
    public static double grade =37 ;
    public static double gradetur =0 ;
    public static double rpm =0 ;
    public static double speed =0 ;
    @Override
    public void runOpMode() throws InterruptedException {
        modules = new RoboModules(hardwareMap);
        modules.launcher.turret.setState(Turret.State.TEST);
        modules.launcher.launcher.setState(Launcher.State.TEST);
        modules.launcher.pivot.setState(Pivot.State.AUTO);
        waitForStart();
        while(opModeIsActive()) {
            modules.update();
            modules.launcher.pivot.setangle(grade);
            modules.launcher.blocator.setState(Blocator.State.OPEN);
            modules.launcher.launcher.setAuto(rpm);
            modules.intake.setManualPower(speed);
            if(gamepad1.y)
                modules.launcher.turret.turret.resetEncoder();
            modules.launcher.turret.setAuto(gradetur*TICKSDEG);




            telemetry.addData("launcher vel",modules.launcher.launcher.Motorvel());
            telemetry.addData("launcherstate",modules.launcher.launcher.getState());
            telemetry.addData("target",modules.launcher.launcher.getTarget());
            telemetry.addData("postur",modules.launcher.turret.getRealCurrentPosition());
            telemetry.addData("postur",modules.launcher.turret.getState());
            telemetry.update();


        }

    }
}
/*
    package org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Launcher;

    import com.acmerobotics.dashboard.config.Config;
    import com.arcrobotics.ftclib.hardware.motors.Motor;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.PIDCoefficients;
    import com.qualcomm.robotcore.hardware.VoltageSensor;

    import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
    import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;

    @Config
    public class Launcher implements IStateBasedModule, IRobotModule {


        public static double TICKS_PER_REV = 28;

        public static double startpos =  0;
        public static final double finalpos = 50 ;
        public static double autopose = 0;
        public static double target = 0; //cm
        public static double currentPosition = startpos;
        public  static double kP = 0.000, kI = 0, kD = 0.00; //TODO Valori PID!!!!
        public  static double kS=0,kV=0; //TODO Valori PID!!!!

        private static boolean ENABLED = true;



        public double manualPower = 0;
        public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

        public State state;
        public EXTRUNMODE runMode;
        VoltageSensor voltage;

        Motor launcher;
        DcMotor motorOne;


        public Launcher(HardwareMap hardwareMap, State initialState) {
            Motor encoder;
            if (!ENABLED) {

                encoder= null;
                motorOne = null;
            } else {
                encoder = new Motor(hardwareMap, "INTAKE", Motor.GoBILDA.BARE);
                launcher = new Motor(hardwareMap, "LANSATOR", Motor.GoBILDA.BARE);
                launcher.encoder = encoder.encoder;
                motorOne = launcher.motor;
                launcher.setRunMode(Motor.RunMode.VelocityControl);
                voltage = hardwareMap.voltageSensor.iterator().next();

            }
            this.state = initialState;
        }

        public void setAuto(double auto) {
            autopose = auto;
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
            return motorOne.getCurrentPosition();
        }

        public double getRealCurrentPosition() {
            return motorOne.getCurrentPosition() / TICKS_PER_REV;
        }

        public boolean atTarget()
        {
            return Math.abs(Motorvel() - target/ TICKS_PER_REV) < 150;
        }

        public  double Revolutions(double ticks) {
            return ticks / TICKS_PER_REV;
        }
        public double Motorvel() {
           return Revolutions(launcher.encoder.getRawVelocity());
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
//            double actualKs = f * 12.0 / voltage.getVoltage();
//            pid = new PIDCoefficients(kP, kI, kD);

            switch (runMode) {
                case PID:
                    launcher.setVeloCoefficients(kP, kI, kD);

                    launcher.setFeedforwardCoefficients(kS, kV);
                     launcher.set(target);

//                    Launcher.setMode(CoolMotor.RunMode.PID);
//                    Launcher.setPIDF(pid, actualKs);
//                    Launcher.calculatePowerFF(Motorvel(), target);
                    break;
                case MANUAL: {

//                    Launcher.setMode(CoolMotor.RunMode.RUN);
//
//                    Launcher.setPower(manualPower);
                    break;
                }
            }


            currentPosition = Motorvel() / TICKS_PER_REV;
//            Launcher.update();

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

 */