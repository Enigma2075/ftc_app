package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "TeleOp drive", group= "learn TeleOp"  )
public class Teleop extends BaseOpMode {

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

            //moves lift and drivetrain
            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, false);
            drivetrain.setPower(powers.rightMotor, powers.leftMotor);

            //goes up with operator y/yellow
            if(gamepad2.y) {
                moveLift(8.1);
            }
            else if(gamepad2.a){
                moveLift(.5);
            }
            else if(gamepad2.x) {
                moveLift(7);
            }

            else if (gamepad2.b) {
                collectInCrater();
            }

            else if (Math.abs(gamepad2.right_stick_y) > .25 ) {
                extendIntake(-gamepad2.right_stick_y);
            }

             else if (gamepad2.right_bumper) {
                suckIntake();
            }


            //exstends out with operator triger/RT
  


            telemetry.addData("liftPosition: ",lift.getCurrentHeight());
            telemetry.update();

        }
    }
}