package org.firstinspires.ftc.teamcode.ROBOT.Modules;

import android.app.Activity;
import android.view.View;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Color {
    public static float gain=6;
    ColorSensor colorSensor;
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    int relativeLayoutId=0;
    // get a reference to the RelativeLayout so we can change the background

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
    View relativeLayout;

    public enum detection{
        PURPLE,
        GREEN,
        UNKOWN;
    }

    public void sinit(HardwareMap hardwareMap,String string)
    {
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
         relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
         relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, string);

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);


    }
    public detection getDetection(Telemetry tele){

        // update previous state variable.
        bPrevState = bCurrState;

        // convert the RGB values to HSV values.
        android.graphics.Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        tele.addData("Clear", colorSensor.alpha());
        tele.addData("Red  ", colorSensor.red());
        tele.addData("Green", colorSensor.green());
        tele.addData("Blue ", colorSensor.blue());
        tele.addData("Hue", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(android.graphics.Color.HSVToColor(0xff, values));
            }
        });
        if(colorSensor.blue()>100 ||colorSensor.green()>100)
        {
            if(colorSensor.green()>colorSensor.blue())
                return detection.GREEN;
            else
                return detection.PURPLE;
        }
        else
            return detection.UNKOWN;
    }
    public detection getDetection(){

        // update previous state variable.
        bPrevState = bCurrState;

        // convert the RGB values to HSV values.
        android.graphics.Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(android.graphics.Color.HSVToColor(0xff, values));
            }
        });
        if(colorSensor.blue()>100 ||colorSensor.green()>100)
        {
            if(colorSensor.green()>colorSensor.blue())
                return detection.GREEN;
            else
                return detection.PURPLE;
        }
        else
            return detection.UNKOWN;
    }
}
