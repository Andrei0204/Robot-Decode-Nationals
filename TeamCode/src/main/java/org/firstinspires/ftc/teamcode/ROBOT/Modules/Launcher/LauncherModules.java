package org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Blocator.Blocator;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Launcher.Launcher;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Launcher.Turret.Turret;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers.IRobotModule;

public class LauncherModules implements IRobotModule {
    public Turret turret;
    public Pivot pivot;
    public Blocator blocator;
    public Launcher launcher;
    public LauncherModules (HardwareMap hardwareMap){
        turret = new Turret(hardwareMap, Turret.State.START);
        pivot = new Pivot(hardwareMap  , Pivot.State.START);
        blocator = new Blocator(hardwareMap  , Blocator.State.CLOSE);
        launcher= new Launcher(hardwareMap, Launcher.State.MANUAL);


    }
    @Override
    public void update() {
        turret.update();
        blocator.update();
        pivot.update();
        launcher.update();
    }
}
