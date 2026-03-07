package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Climb {
    double power = 0, timer = 0;
    ElapsedTime time;
    Servo C1, C2;

    public Climb(HardwareMap hardwareMap) {
        C1 = hardwareMap.get(Servo.class, "CLIMB1");
        C2 = hardwareMap.get(Servo.class, "CLIMB2");
        time = new ElapsedTime();
    }

    public void setPower(double power, double timer) {
        if (power != this.power) {
            this.power = power;
            time.reset();
            this.timer = timer;
        }
    }

    public void update() {
        if (timer != 0) {
            if (time.milliseconds() < timer) {
                if (C1.getPosition() != power) {


                    C1.setPosition(power);
                    C2.setPosition(power);
                }
            } else
                timer = 0;
        }
        if (timer == 0 && C1.getPosition() != 0.53) {
            C1.setPosition(0.53);
            C2.setPosition(0.53);
        }
    }

}
