package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="CheesyDrive", group="TeleOp")

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
        double throdle = cheesyDrive.handleDeadband(gamepad1.left_stick_y*-1, .1);
        boolean quickTurn = throdle ==0;
        DriveSignal powers = cheesyDrive.cheesyDrive(throdle, gamepad1.right_stick_x, quickTurn);
        robot.rightDrive.setPower(powers.rightMotor);
        robot.leftDrive.setPower(powers.leftMotor);

    }
}
