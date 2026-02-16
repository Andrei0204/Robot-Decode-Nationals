package org.firstinspires.ftc.teamcode.ROBOT.testers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.ROBOT.Modules.Follower;
import org.firstinspires.ftc.teamcode.ROBOT.roboDrive.math.Pose;

import java.util.Locale;

@Config
@Configurable
@TeleOp
public class backnforth extends OpMode {

    ElapsedTime timer;
    public static double x = -6, y = -148;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Compu
    Follower follower;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        odo.setOffsets(x, y, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        follower = new Follower(hardwareMap, odo);
        follower.setTarget(new Pose( 0,0 ,Math.toRadians(90)));
    }

    @Override
    public void start() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));

    }

    @Override
    public void loop() {
        if (gamepad1.a)
            follower.setTarget(new Pose(40, 0, Math.toRadians(90)));
        if (gamepad1.b)
            follower.setTarget(new Pose(0, 40, Math.toRadians(90)));
        if (gamepad1.x)
            follower.setTarget(new Pose(40, 40, Math.toRadians(90)));
        if (gamepad1.y)
            follower.setTarget(new Pose(0, 0,    Math.toRadians(90)));
        if(gamepad1.dpad_down)
            follower.setTarget(new Pose(40, 0, Math.toRadians(0)));
        follower.update();
        odo.update();



        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("target", follower.getTarget());
        telemetry.update();
    }
}



