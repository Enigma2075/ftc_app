package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareLift {    /* Public OpMode members. */
    private DcMotor motor;
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
        motor = hwMap.get(DcMotor.class, "lift");

        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }



    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setPower(double liftMotorPower) {
        motor.setPower(liftMotorPower);
    }
}
