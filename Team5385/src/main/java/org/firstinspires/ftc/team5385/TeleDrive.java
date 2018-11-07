package org.firstinspires.ftc.team5385;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleDrive", group = "Teleop")
public class TeleDrive extends BigAutoBase {
    CheesyDrive cheesyDrive = null;

    enum LiftMode {UP, DOWN, STOP}

    @Override
    public void runOpMode() {

        cheesyDrive = new CheesyDrive();
        drivetrain.init(hardwareMap);
        lift.init(hardwareMap);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        LiftMode currentLiftMode = LiftMode.STOP;

        while(opModeIsActive()) {

            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y*-1, gamepad1.right_stick_x, gamepad1.left_stick_y==0);
            drivetrain.setPower(powers.leftMotor, powers.rightMotor);


            if (gamepad2.dpad_up && currentLiftMode != LiftMode.UP){
                lift.setPower(1);
                currentLiftMode = LiftMode.UP;
            }
            else if (gamepad2.dpad_down && currentLiftMode != LiftMode.DOWN){
                lift.setPower(-1);
                currentLiftMode = LiftMode.DOWN;
            }
            else if(!gamepad2.dpad_down && !gamepad2.dpad_up && currentLiftMode != LiftMode.STOP) {
                lift.setPower(0);
                currentLiftMode = LiftMode.STOP;
            }


            telemetry.addData("liftPosition", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}