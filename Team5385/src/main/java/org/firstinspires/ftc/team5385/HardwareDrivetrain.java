package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareDrivetrain {
    /* Public OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    //declaring the jumper motor
    private Servo jumper = null;


    /* local OpMode members. */
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    /* Constructor */
    public HardwareDrivetrain() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        jumper = hwMap.get(Servo.class, "driveJumper");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        ServoImplEx spark = (ServoImplEx) jumper;

        spark.setPwmRange(new PwmControl.PwmRange(500, 2500));

        jumper.setPosition(0.5);

        jumper.setDirection(Servo.Direction.FORWARD);
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    public void setMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
    }
    public void setTarget(int leftTarget, int rightTarget){
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);

    }
    public void setPower(double leftPower, double rightPower){
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void setJumperPower(double power){
        jumper.setPosition(.5+.5*power);
    }
    public double getJumperPower(){
        return (jumper.getPosition()-.5)*2;
    }

    public int getCurrentRightPosition(){
        return rightDrive.getCurrentPosition();
    }

    public int getCurrentLeftPosition(){
        return leftDrive.getCurrentPosition();
    }

    public void setTargetPosition(int rightTragetPosition,int leftTargetPosition){
        leftDrive.setTargetPosition(leftTargetPosition);
        rightDrive.setTargetPosition(rightTragetPosition);
    }

    public boolean isLeftBusy(){
        return leftDrive.isBusy();
    }

    public boolean isRightBusy(){
        return rightDrive.isBusy();
    }

}
