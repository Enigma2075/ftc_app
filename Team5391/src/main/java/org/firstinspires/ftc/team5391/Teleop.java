package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

public class Teleop extends OpMode {

    HardwareDrivetrain drivetrain = null;
    CheesyDrive cheesyDrive = null;
    
    @Override
    public void init() {
        drivetrain = new HardwareDrivetrain();
        drivetrain.init(hardwareMap);
        cheesyDrive = new CheesyDrive();
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop() {
        DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, false);
        drivetrain.setPower(powers.rightMotor, powers.leftMotor);
    }
}
