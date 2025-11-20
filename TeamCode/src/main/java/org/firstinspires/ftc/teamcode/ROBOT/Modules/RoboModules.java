package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.LauncherModules;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;

public class RoboModules implements IRobotModule {
     public LauncherModules launcher;
     public Intake intake;
     public HardwareMap hardwareMap;
    public RoboModules(HardwareMap hardwareMap)
    {
        this.hardwareMap=hardwareMap;
        launcher= new LauncherModules(hardwareMap);
        intake= new Intake(hardwareMap, Intake.State.MANUAL);
    }

    @Override
    public void update()  {
        launcher.update();
        intake.update();
    }
}
