package org.firstinspires.ftc.team5385;

import android.hardware.Sensor;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareColorSensor {

    private Servo servo;
    private ColorSensor sensor;

    HardwareMap hwMap = null;

    public HardwareColorSensor(){
    }

    public void init(HardwareMap hwMap){

        this.hwMap = hwMap;

        servo = hwMap.get(Servo.class, "colorServo");
        sensor = hwMap.get(ColorSensor.class, "colorSensor");
        sensor.resetDeviceConfigurationForOpMode();
    }

    public void setPosition(double pos){
        servo.setPosition(pos);
    }
    public double getPosition(){
        return servo.getPosition();
    }


    public double getColor(){
       return sensor.blue();
    }
}
