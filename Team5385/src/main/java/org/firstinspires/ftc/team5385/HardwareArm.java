package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HardwareArm {    /* Public OpMode members. */
    private static final double MAX_POT = 3.3 * (9300.0/9400.0);
    private static final double MIN_POT = 3.3 * (300.0/9400.0);

    private Servo bucket;
    private DcMotor shoulder;
    private DcMotor elbow;
    private AnalogInput shoulderPot;
    private AnalogInput elbowPot;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public HardwareArm() {
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        shoulderPot = hwMap.get(AnalogInput.class, "shoulder_pot");
        shoulder = hwMap.get(DcMotor.class, "shoulder");
        elbowPot = hwMap.get(AnalogInput.class, "elbow_pot");
        elbow = hwMap.get(DcMotor.class, "elbow");

        bucket = hwMap.get(Servo.class, "bucket");

        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setPower(0);
        elbow.setPower(0);


    }

    public void setElbowPower(double power) {
        elbow.setPower(power);
    }

    //public void set

    public void setPower(double liftMotorPower) {
       double position = 0.5 + .5 * liftMotorPower;
       if (position > 1){
           position = 1;
       }
       else if (position < 0){
           position = 0;
       }
       //motor.setPosition(position);
    }

    //public double getCurrentPosition(){
    //    return pot.getVoltage();
    //}

    //public double getError(double target){
    //    return getCurrentPosition()-target;
    //}
    //public double getPosition(){
    //    return motor.getPosition();
    //}

}
