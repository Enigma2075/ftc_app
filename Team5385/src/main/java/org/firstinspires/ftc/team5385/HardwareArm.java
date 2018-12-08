package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class HardwareArm {    /* Public OpMode members. */
    private static final double MAX_POT = 3.3 * (9300.0/9400.0);
    private static final double MIN_POT = 3.3 * (300.0/9400.0);

    private Servo bucket;
    private Servo score;
    private DcMotor shoulder;
    private DcMotor elbow;
    private AnalogInput shoulderPot;
    private AnalogInput elbowPot;

    private static final double SHOULDER_P_COEFF = 8;
    private static final double ELBOW_P_COEFF = 10;

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
        score = hwMap.get(Servo.class, "score");

        shoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setPower(0);
        elbow.setPower(0);


    }

    public void setElbowPower(double power) {
        elbow.setPower(power);
    }

    public void setShoulderPower(double power){
        shoulder.setPower(power);
    }

    public double getElbowPower() {
        return elbow.getPower();
    }

    public double getShoulderPower(){
        return shoulder.getPower();
    }

    public void goToTarget(double shoulderTarget, double elbowTarget){
        double shoulderError = getShoulderError(shoulderTarget);
        double elbowError = getElbowError(elbowTarget);

        double elbowPower = elbowError * ELBOW_P_COEFF;
        double shoulderPower = shoulderError * SHOULDER_P_COEFF;

        if(Math.abs(elbowPower) > .001) {
            setElbowPower(elbowPower);
        }
        else {
            setElbowPower(0);
        }

        if(Math.abs(shoulderPower) > .001) {
            setShoulderPower(shoulderPower);
        }
        else {
            setShoulderPower(0);
        }
    }

    public double shoulderPosition(){
        return shoulderPot.getVoltage();
    }

    public double elbowPosition(){
        return elbowPot.getVoltage();
    }

    public double getElbowError(double target){
        return elbowPosition() - target;
    }

    public double getShoulderError(double target){
        return shoulderPosition() - target;
    }

    public void setServoPos(double pos){
        bucket.setPosition(pos);
    }

    public void setScorePos(double pos) {
        score.setPosition(pos);
    }

    public void stopArm(){
        setElbowPower(0);
        setShoulderPower(0);
    }

    public double getservoPos(){
        return bucket.getPosition();
    }

}
