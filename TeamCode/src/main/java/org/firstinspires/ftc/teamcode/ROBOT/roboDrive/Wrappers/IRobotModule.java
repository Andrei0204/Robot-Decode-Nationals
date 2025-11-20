package org.firstinspires.ftc.teamcode.ROBOT.roboDrive.Wrappers;

public interface IRobotModule {
    public static boolean ENABLED = false;
    default void initUpdate() {}
    default void atStart() {}
    void update() throws InterruptedException;
    default void emergencyStop() {}
}
