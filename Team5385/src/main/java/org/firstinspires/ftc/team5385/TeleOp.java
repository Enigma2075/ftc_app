package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="CheesyDrive", group="TeleOp")

public class TeleOp extends OpMode {

    HardwareDrivetrain robot = null;
    CheesyDrive cheesyDrive = null;

    @Override
    public void init() {

        robot = new HardwareDrivetrain();
        robot.init(hardwareMap);
        cheesyDrive = new CheesyDrive();
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void loop() {
        double throdle = cheesyDrive.handleDeadband(gamepad1.left_stick_y*-1, .1);
        boolean quickTurn = throdle ==0;
        DriveSignal powers = cheesyDrive.cheesyDrive(throdle, gamepad1.right_stick_x, quickTurn);
        robot.setPower(powers.rightMotor, powers.leftMotor);

    }
}
