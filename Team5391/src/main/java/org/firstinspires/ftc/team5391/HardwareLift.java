package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareLift {
    private DcMotor liftMotor = null;
    private DcMotor liftMotor2 = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareLift() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        liftMotor = hwMap.get(DcMotor.class, "LeftMotor");
        liftMotor2 = hwMap.get(DcMotor.class, "LeftMotor2");

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(0);
        liftMotor2.setPower(0);


    }

    public void setMode(DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
        liftMotor2.setMode(mode);
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setBothTargetPosition(int targetPosition) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor2.setTargetPosition(targetPosition);
    }


    public int getBothCurrentPosition() {
        return liftMotor2.getCurrentPosition();
    }

}














