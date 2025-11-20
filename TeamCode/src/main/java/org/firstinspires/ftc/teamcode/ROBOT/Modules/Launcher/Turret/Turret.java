    package org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret;

    import com.acmerobotics.dashboard.config.Config;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.PIDCoefficients;

    import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.CoolMotor;
    import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;
    import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IStateBasedModule;

    @Config
    public class Turret implements IStateBasedModule, IRobotModule {


        public static final double TICKS435 = 384.5;    //test
        public static final double TURTICKS = TICKS435 *8.6;    //test
        public static double TICKSDEG = TURTICKS/360;

        public static double startpos =  0* TICKSDEG;
        public static final double finalpos = 50 * TICKSDEG;
        public static double autopose = 0;
        public static double target = 0; //cm
        public static double currentPosition = startpos;
        public  static double kP = 0.0078, kI = 0.0007, kD = 0.00015; //TODO Valori PID!!!!

        private static boolean ENABLED = true;
        public CoolMotor turret = null;


        public double manualPower = 0;
        public PIDCoefficients pid = new PIDCoefficients(kP, kI, kD);

        public State state;
        public EXTRUNMODE runMode;



        public Turret(HardwareMap hardwareMap, State initialState) {
            if (!ENABLED) {


                turret = null;
            } else {

                turret = new CoolMotor(hardwareMap.get(DcMotorEx.class, "TURETA"), CoolMotor.RunMode.RUN,true);

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
            return turret.getCurrentPosition();
        }

        public double getRealCurrentPosition() {
            return turret.getCurrentPosition() / TICKSDEG;
        }

        public boolean atTarget()
        {
            return Math.abs(getRealCurrentPosition() - target/ TICKSDEG) < 5;
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
            if(auto<0)
                auto=0;
            if(auto>220*TICKSDEG)
                auto=220*TICKSDEG;
            autopose = auto;
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
                    case TEST:
                        setTarget(autopose);
                        break;

                }
            }
        }

        @Override
        public void updateHardware() {
            pid = new PIDCoefficients(kP, kI, kD);

            switch (runMode) {
                case PID:
                    turret.setMode(CoolMotor.RunMode.PID);


                    turret.setPIDF(pid, 0);




                    turret.calculatePowerFF(turret.getCurrentPosition(), target);
                    break;
                case MANUAL: {

                    turret.setMode(CoolMotor.RunMode.RUN);

                    turret.setPower(manualPower);
                    break;
                }
            }


            currentPosition = turret.getCurrentPosition() / TICKSDEG;
            turret.update();

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
