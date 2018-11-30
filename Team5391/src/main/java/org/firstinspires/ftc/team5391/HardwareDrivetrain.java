package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareDrivetrain {
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 30.0/22.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    /* Public OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwareDrivetrain() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "LeftDrive");
        rightDrive = hwMap.get(DcMotor.class, "RightDrive");
        leftDrive2 = hwMap.get(DcMotor.class, "LeftDrive2");
        rightDrive2 = hwMap.get(DcMotor.class, "RightDrive2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setPower(0, 0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Mode
    public void setRightMode(DcMotor.RunMode mode) {
        rightDrive.setMode(mode);
        rightDrive2.setMode(mode);
    }

    public void setLeftMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        leftDrive2.setMode(mode);
    }

    public void setMode(DcMotor.RunMode rightMode, DcMotor.RunMode leftMode) {
        setLeftMode(leftMode);
        setRightMode(rightMode);
    }

    public void setMode(DcMotor.RunMode mode) {
        setLeftMode(mode);
        setRightMode(mode);
    }

    // Power
    public void setRightPower(double power) {
        rightDrive.setPower(power);
        rightDrive2.setPower(power);
    }

    public void setLeftPower(double power) {
        leftDrive.setPower(power);
        leftDrive2.setPower(power);
    }

    public void setPower(double rightPower, double leftPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setPower(double power) {
        setRightPower(power);
        setLeftPower(power);
    }

    // Target Position
    public void setRightTargetPosition(int targetPosition) {
        rightDrive.setTargetPosition(targetPosition);
        rightDrive2.setTargetPosition(targetPosition);
    }

    public void setLeftTargetPosition(int targetPosition) {
        leftDrive.setTargetPosition(targetPosition);
        leftDrive2.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(int rightTargetPosition, int leftTargetPosition) {
        setLeftTargetPosition(leftTargetPosition);
        setRightTargetPosition(rightTargetPosition);
    }

    public void setTargetPosition(int targetPosition) {
        setLeftTargetPosition(targetPosition);
        setRightTargetPosition(targetPosition);
    }

    // Current Position
    public int getRightCurrentPosition() {
        return rightDrive.getCurrentPosition();
    }

    public int getLeftCurrentPosition() {
        return leftDrive.getCurrentPosition();
    }

    // Is Busy
    public boolean isRightBusy() {
        return rightDrive.isBusy();
    }

    public boolean isLeftBusy() {
        return leftDrive.isBusy();
    }
}

