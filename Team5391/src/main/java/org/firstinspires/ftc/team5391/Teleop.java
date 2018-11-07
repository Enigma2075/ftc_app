package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "TeleOp drive", group= "learn TeleOp"  )
public class Teleop extends BaseOpMode {
    CheesyDrive cheesyDrive = null;

    @Override
    public void runOpMode() {
        isAuto = false;
        super.runOpMode();

        cheesyDrive = new CheesyDrive();

        waitForStart();
        while (opModeIsActive()) {
            //moves lift and drivetrain
            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, false);
            drive(powers);

            //goes up with operator y/yellow
            if(gamepad2.y) {
                moveLift(8.1);
            }
            else if(gamepad2.a){
                moveLift(.5);
            }
            else if(gamepad2.x) {
                moveLift(7.25);
            }

            if (gamepad2.b && !isInCrater()) {
                collectInCrater();
            }
            else if (gamepad2.b && Math.abs(gamepad2.right_stick_y) > .25 ) {
                moveIntake(-gamepad2.right_stick_y);
            }
            else {
                moveIntake(0);
            }

            sendTelemetry();
            telemetry.update();
        }
    }
}