package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HardwareLift {    /* Public OpMode members. */
    private static final double MAX_POT = 3.3 * (9300.0/9400.0);
    private static final double MIN_POT = 3.3 * (300.0/9400.0);

    private Servo motor;
    private AnalogInput pot;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareLift() {
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        pot = hwMap.get(AnalogInput.class, "lift_pot");
        motor = hwMap.get(Servo.class, "lift");

        ServoImplEx spark = (ServoImplEx) motor;

        spark.setPwmRange(new PwmControl.PwmRange(500, 2500));

        motor.setPosition(0.5);

        motor.setDirection(Servo.Direction.FORWARD);
    }

    public void setPower(double liftMotorPower) {
       double position = 0.5 + .5 * liftMotorPower;
       if (position > 1){
           position = 1;
       }
       else if (position < 0){
           position = 0;
       }
       motor.setPosition(position);
    }

    public double getCurrentPosition(){
        return pot.getVoltage();
    }

    public double getError(double target){
        return getCurrentPosition()-target;
    }
    public double getPosition(){
        return motor.getPosition();
    }

}
