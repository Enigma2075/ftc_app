package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "TeleDrive", group = "Teleop")
public class Tele_Op extends BigAutoBase {

    HardwareBot robot = null;
    CheesyDrive cheesyDrive = null;

    @Override
    public void runOpMode() {

        robot = new HardwareBot();
        robot.init(hardwareMap);
        cheesyDrive = new CheesyDrive();
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()) {

            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y*-1, gamepad1.right_stick_x, gamepad1.left_stick_y==0);
            robot.rightDrive.setPower(powers.rightMotor);
            robot.leftDrive.setPower(powers.leftMotor);
        }
    }
}
