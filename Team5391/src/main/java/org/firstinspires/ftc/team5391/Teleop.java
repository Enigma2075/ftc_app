package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
@TeleOp(name= "TeleOp drive", group= "learn TeleOp"  )
public class Teleop extends BaseAutonomous {

    HardwareDrivetrain drivetrain = null;
    CheesyDrive cheesyDrive = null;
    HardwareLift lift = null;

    @Override
    public void runOpMode() {
        drivetrain = new HardwareDrivetrain();
        drivetrain.init(hardwareMap);
        cheesyDrive = new CheesyDrive();
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, false);
            drivetrain.setPower(powers.rightMotor, powers.leftMotor);
            lift.setPower(gamepad2.right_stick_y);
            telemetry.addData("liftPosition: ",lift.getRightCurrentPosition() );
            telemetry.update();
            }
    }
}