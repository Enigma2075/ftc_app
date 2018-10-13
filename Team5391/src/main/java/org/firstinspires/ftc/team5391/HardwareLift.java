package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareLift {
    private DcMotor liftMotor=null;
    private DcMotor liftMotor2=null;

    HardwareMap hwMap= null;
    private ElapsedTime peried = new  ElapsedTime();

    public HardwareLift(){

    }
    public void init(HardwareMap ahwMap) {
    hwMap=ahwMap;

    liftMotor = hwMap.get(DcMotor.class, "LeftMotor");
    liftMotor2 =hwMap.get(DcMotor.class, "Leftmotor2");





    }
}

















