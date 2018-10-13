package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareLift {
    private DcMotor liftMotor = null;
    private DcMotor liftMotor2 = null;

    HardwareMap hwMap = null;
    private ElapsedTime peried = new ElapsedTime();

    public HardwareLift() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        liftMotor = hwMap.get(DcMotor.class, "LeftMotor");
        liftMotor2 = hwMap.get(DcMotor.class, "Leftmotor2");


    }

    public void setRightMode(DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
        liftMotor2.setMode(mode);
    }

    public void setLeftMode(DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
        liftMotor2.setMode(mode);
    }

    public void setMode(DcMotor.RunMode rightMode, DcMotor.RunMode leftMode) {
        setRightMode(rightMode);
        setLeftMode(leftMode);
    }

    public void setRightPower(double power) {
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setLeftPower(double power) {
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setPower(double rightPower, double leftPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setPower(double power) {
        setRightPower(power);
        setLeftPower(power);
    }

    public void setRightTargetPosition(int targetPosition) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor2.setTargetPosition(targetPosition);
    }

    public void setLeftTargetPosition(int targetPosition) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor2.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(int rightTargetPosition, int leftTargetPosition) {
        setLeftTargetPosition(leftTargetPosition);
        setRightTargetPosition(rightTargetPosition);
    }

    public void setTargetPosition(int targetPosition) {
        setLeftTargetPosition(targetPosition);
        setRightTargetPosition(targetPosition);
    }

    public int getRightCurrentPosition() { return liftMotor2.getCurrentPosition();}

    public int getLeftCurrentPosition() { return liftMotor.getCurrentPosition();}

    public boolean isRightBusy() { return liftMotor.isBusy();}

    public boolean isLeftBusy() { return liftMotor2.isBusy();}
}














