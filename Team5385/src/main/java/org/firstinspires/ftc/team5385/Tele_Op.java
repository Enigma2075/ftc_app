package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Tele_Op extends OpMode {

    HardwareBot robot = null;
    CheesyDrive cheesyDrive = null;

    @Override
    public void init() {

        robot = new HardwareBot();
        robot.init(hardwareMap);
        cheesyDrive = new CheesyDrive();
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y, gamepad2.right_stick_x, false);
        robot.rightDrive.setPower(powers.rightMotor);
        robot.leftDrive.setPower(powers.leftMotor);

    }
}
