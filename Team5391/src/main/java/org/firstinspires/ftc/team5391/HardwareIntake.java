package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

public class HardwareIntake {
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double SPOOL_CIRCUMFERENCE_INCHES = 1.375 * Math.PI;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (SPOOL_CIRCUMFERENCE_INCHES);

    static final double MAX_EXTENSION = 27;
    static final double MIN_EXTENSION = 0;


    private DcMotor extensionMotor = null;
    private DcMotor intakeMotor = null;
    private AnalogInput sensor =null;

    private CRServo rightPivot = null;
    private CRServo leftPivot = null;

    private HardwareMap hwMap = null;

    static final double P_PIVOT_COEFF=2.5;

    public enum IntakePivot {
        NONE(-1.0), BALLS(1.05), BLOCKS(.88), TRANSFER(2.6), STORE(1.6), IN_DUMP(2.47);

        private final double position;
        IntakePivot(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public HardwareIntake() {
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        extensionMotor = hwMap.get(DcMotor.class, "extensionMotor");
        extensionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightPivot= hwMap.get(CRServo.class, "rightPivot");
        leftPivot= hwMap.get(CRServo.class, "leftPivot");
        sensor = hwMap.get(AnalogInput.class,"sensor");

        ((CRServoImplEx) rightPivot).setPwmRange(new PwmControl.PwmRange(500, 2500));
        ((CRServoImplEx) leftPivot).setPwmRange(new PwmControl.PwmRange(500, 2500));

        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setExtensionPower(0);
        setIntakePower(0);
    }

    public void setIntakePower(double power) {
        if(intakeMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        intakeMotor.setPower(power);
    }

    public void movePivot(IntakePivot position){
        if(position == IntakePivot.NONE) {
            return;
        }

        if(isPivotBusy(position)){
            double power = getPivotError(position)*P_PIVOT_COEFF;
            if(power>1)
                power=1;
            else if(power<-1)
                power=-1;

            leftPivot.setPower(-power);
            rightPivot.setPower(power);
        }
    }

    public double getPivotPosition() {
        return sensor.getVoltage();
    }

    public boolean isPivotBusy(IntakePivot position) {
        return Math.abs(getPivotError(position))>0.001;
    }


    public void setExtensionPosition(double targetExtension) {
        double target = targetExtension;
        if(targetExtension > MAX_EXTENSION) {
            target = MAX_EXTENSION;
        }
        if(targetExtension < MIN_EXTENSION) {
            target = MIN_EXTENSION;
        }

        if(extensionMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(getExtensionTarget() != target) {
            extensionMotor.setTargetPosition((int)(target * COUNTS_PER_INCH));

            double targetPower = 1;
            if(targetExtension < getCurrentExtension()) {
                targetPower = 1;
            }
            extensionMotor.setPower(targetPower);
        }
    }

    public double getCurrentExtension() {
        return (double) extensionMotor.getCurrentPosition() / COUNTS_PER_INCH;
    }

    public boolean isExtensionBusy() {
        return extensionMotor.isBusy();
    }

    public double getExtensionTarget() { return extensionMotor.getTargetPosition();}

    public void resetEncoders() {
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setExtensionPower(double power) {
        if(extensionMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double targetPower = power;
        if(targetPower < 0) {
            targetPower *= 1;
        }
        extensionMotor.setPower(targetPower);
    }

    public DcMotor.RunMode getExtensionMode() {
        return extensionMotor.getMode();
    }

    private void setExtensionMode(DcMotor.RunMode mode) {
        intakeMotor.setMode(mode);
    }

    public double getPivotError(IntakePivot target) {
        return getPivotPosition()- target.getPosition();
    }
}














