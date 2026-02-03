package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ROBOT.LTur;
import org.firstinspires.ftc.teamcode.ROBOT.Spinner;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;

public class RoboModules implements IRobotModule {

    public LTur lTur=null;
    public Spinner spinner=null;
    public Intake intake=null;


    public HardwareMap hardwareMap;
    public RoboModules(HardwareMap hardwareMap)
    {
        this.hardwareMap=hardwareMap;
        lTur= new LTur(hardwareMap, LTur.State.START);
        spinner= new Spinner(hardwareMap, Spinner.State.START);
        intake= new Intake(hardwareMap, Intake.State.MANUAL);
    }

    @Override
    public void update()  {

        lTur.update();
        spinner.update();
        intake.update();
    }
}
