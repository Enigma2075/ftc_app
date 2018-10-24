package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareLift {
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 30.0/22.0;     // This is < 1.0 if geared UP
    static final double SPOOL_CIRCUMFERENCE_INCHES = 1.27;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (SPOOL_CIRCUMFERENCE_INCHES);

    static final double MAX_HEIGHT = 9.5;

    private DcMotor liftMotor = null;
    private DcMotor liftMotor2 = null;

    private HardwareMap hwMap = null;

    public HardwareLift() { }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        liftMotor = hwMap.get(DcMotor.class, "LiftMotor");
        liftMotor2 = hwMap.get(DcMotor.class, "LiftMotor2");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPower(0);
    }

    public void setMode(DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
        liftMotor2.setMode(mode);
    }

    public void setPower(double power) {
        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }

    public void setTargetPosition(double targetHeight) {
        if(targetHeight > MAX_HEIGHT) {
            targetHeight = MAX_HEIGHT;
        }

        int targetPosition = (int)(targetHeight * COUNTS_PER_INCH);
        liftMotor.setTargetPosition(targetPosition);
        liftMotor2.setTargetPosition(targetPosition);
    }


    public double getCurrentHeight() {
        return (double)liftMotor.getCurrentPosition() * COUNTS_PER_INCH;
    }

    public boolean isBusy() {
        return liftMotor.isBusy();
    }
}













