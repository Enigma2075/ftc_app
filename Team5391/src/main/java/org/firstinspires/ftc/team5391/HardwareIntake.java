package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ResourceBundle;

public class HardwareIntake {
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double SPOOL_CIRCUMFERENCE_INCHES = 4.71;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (SPOOL_CIRCUMFERENCE_INCHES);

    static final double MAX_EXTENSION = 30;

    private DcMotor extensionMotor = null;
    private DcMotor intakeMotor = null;
    private AnalogInput sensor =null;

    private CRServo rightPivot = null;
    private CRServo leftPivot = null;

    private HardwareMap hwMap = null;

    public HardwareIntake() {
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        extensionMotor = hwMap.get(DcMotor.class, "extensionMotor");
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightPivot= hwMap.get(CRServo.class, "rightPivot");
        leftPivot= hwMap.get(CRServo.class, "leftPivot");
        sensor = hwMap.get(AnalogInput.class,"sensor");

        setIntakeMode(DcMotor.RunMode.RUN_TO_POSITION);
        setExtensionMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setExtensionPower(0);
        setintakePower(0);
    }

    public void setExtensionMode(DcMotor.RunMode mode) {
        extensionMotor.setMode(mode);
    }

    public void setIntakeMode(DcMotor.RunMode mode) {
        intakeMotor.setMode(mode);
    }

    public void setExtensionPower(double power) {
        extensionMotor.setPower(power);
    }

    public void setintakePower(double power) {
        extensionMotor.setPower(power);
    }

    public void extend(double distance) {
        setExtensionPosition(distance);
        setExtensionPower(1);
    }

    public void suckinIntake() {
        setIntakeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setintakePower(1);
    }

    public double potPosision(){
        return sensor.getVoltage();
    }

    public void setpivet(double power){
        double PWR= .5+ .5*power;
        leftPivot.setPower(PWR);
        rightPivot.setPower(PWR);
    }


    public void setExtensionPosition(double targetExtension) {
        if (targetExtension > MAX_EXTENSION) {
            targetExtension = MAX_EXTENSION;
        }

        int targetPosition = (int) (targetExtension * COUNTS_PER_INCH);
        extensionMotor.setTargetPosition(targetPosition);
    }


    public double getCurrentExtension() {
        return (double) extensionMotor.getCurrentPosition() * COUNTS_PER_INCH;
    }

    public DcMotor.RunMode getExtensionMode() {
        return extensionMotor.getMode();
    }

    public boolean isBusy() {
        return extensionMotor.isBusy();
    }
}














