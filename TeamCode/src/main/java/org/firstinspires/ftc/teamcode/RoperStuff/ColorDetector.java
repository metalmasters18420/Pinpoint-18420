package org.firstinspires.ftc.teamcode.RoperStuff;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import android.graphics.Color;

public class ColorDetector {
    ColorSensor colorSensorTop = null;
    ColorSensor colorSensorBottom = null;
    SampleColor TopSampleColorDetected = SampleColor.UNKNOWN;
    SampleColor BottomSampleColorDetected = SampleColor.UNKNOWN;
    private final double RED_LOWER_BOUND = 300;
    private final double RED_UPPER_BOUND = 30;
    private final double YELLOW_LOWER_BOUND = 45;
    private final double YELLOW_UPPER_BOUND = 120;
    private final double BLUE_LOWER_BOUND = 180;
    private final double BLUE_UPPER_BOUND = 270;


    public enum SampleColor {RED,YELLOW,BLUE,UNKNOWN};

    double SCALE_FACTOR = 255;
    public ColorDetector(HardwareMap hwmap){
        colorSensorTop = hwmap.get(ColorSensor.class,"color_top");
        colorSensorBottom = hwmap.get(ColorSensor.class,"color_bottom");
    }


    /*
    public Color readSampleColor(){
        if (){
            return Color.YELLOW;
        }
        else if(){
            return Color.BLUE;
        }
        else if(){
            return Color.RED;
        }
        else{
            return Color.UNKNOWN;
        }
    }
     */



    public SampleColor updateIndividualSensor(ColorSensor sensor){
        float[] hsvValues = {0F, 0F, 0F};
        double hue;

        Color.RGBToHSV((int) (sensor.red() * SCALE_FACTOR),
                (int) (sensor.green() * SCALE_FACTOR),
                (int) (sensor.blue() * SCALE_FACTOR),
                hsvValues);
        hue = hsvValues[0];

        if ((hue > RED_LOWER_BOUND) || hue < RED_UPPER_BOUND){
            return SampleColor.RED;
        }
        else if (hue > YELLOW_LOWER_BOUND && hue < YELLOW_UPPER_BOUND){
            return SampleColor.YELLOW;
        }
        else if (hue > BLUE_LOWER_BOUND && hue < BLUE_UPPER_BOUND){
            return SampleColor.BLUE;
        }
        else {
            return SampleColor.UNKNOWN;
        }
    }

    public SampleColor detectedColor(){
        SampleColor topColor = updateIndividualSensor(colorSensorTop);
        SampleColor bottomColor = updateIndividualSensor(colorSensorBottom);
        if (topColor == bottomColor){
            return topColor;
        }
        else{
            return SampleColor.UNKNOWN;
        }
    }

    public float[] readHSV(ColorSensor sensor){
        float[] hsvValues = {0F, 0F, 0F};

        Color.RGBToHSV((int) (sensor.red() * SCALE_FACTOR),
                (int) (sensor.green() * SCALE_FACTOR),
                (int) (sensor.blue() * SCALE_FACTOR),
                hsvValues);



        return hsvValues;
    }
}
