package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ROBOT.LTur;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.DistanceC;
import org.firstinspires.ftc.teamcode.ROBOT.Spinner;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;

public class RoboModules implements IRobotModule {

    public LTur lTur=null;
    public Spinner spinner=null;
    public Intake intake=null;
    public Pivot pivot=null;
    public Launcher launcher=null;
    Color color= new Color();
    Color color2= new Color();
    Color color3= new Color();
    Distance distance= new Distance();
    Distance distance2= new Distance();
    Distance distance3= new Distance();

    DistanceC distancec= new DistanceC();
    DistanceC distance2c= new DistanceC();
    DistanceC distance3c= new DistanceC();





    public HardwareMap hardwareMap;
    public RoboModules(HardwareMap hardwareMap)
    {
        this.hardwareMap=hardwareMap;
        lTur= new LTur(hardwareMap, LTur.State.START);
        spinner= new Spinner(hardwareMap, Spinner.State.START);
        intake= new Intake(hardwareMap, Intake.State.MANUAL);
        pivot= new Pivot(hardwareMap, Pivot.State.AUTO);
        launcher= new Launcher(hardwareMap, Launcher.State.TEST);
//        color.sinit(hardwareMap,"C1");
//        color2.sinit(hardwareMap,"C2");
//        color3.sinit(hardwareMap,"C3");

    }

    public void  sensorstart(){
        distancec.sinit(hardwareMap,"C1");
        distance2c.sinit(hardwareMap,"C2");
        distance3c.sinit(hardwareMap,"C3");
        distance.sinit(hardwareMap,"D1");
        distance2.sinit(hardwareMap,"D2");
        distance3.sinit(hardwareMap,"D3");
    }
    @Override
    public void update()  {

        lTur.update();
        spinner.update();
        intake.update();
        pivot.update();
        launcher.update();
    }
}
