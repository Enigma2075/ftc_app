package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleDrive", group = "Teleop")
public class TeleDrive extends BigAutoBase {
    CheesyDrive cheesyDrive = null;

    @Override
    public void runOpMode() {

        cheesyDrive = new CheesyDrive();
        drivetrain.init(hardwareMap);
        lift.init(hardwareMap);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {

            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y*-1, gamepad1.right_stick_x, gamepad1.left_stick_y==0);
            drivetrain.setPower(powers.leftMotor, powers.rightMotor);

            lift.setPower(gamepad2.left_stick_y*-1);
        }
    }
}