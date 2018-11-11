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
        arm.init(hardwareMap);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        LiftMode currentLiftMode = LiftMode.STOP;
        ArmPosition currentArmPosition = ArmPosition.HOME;
        while(opModeIsActive()) {

            /** Code To Run The Drivetrain In Tele-Op **/
            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y*-1, gamepad1.right_stick_x, gamepad1.left_stick_y==0);
            drivetrain.setPower(powers.leftMotor, powers.rightMotor);

            /** Code To Run The Lift In Tele-Op **/
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

            if (currentLiftMode == LiftMode.UP && lift.getCurrentPosition() < 0.2){
                lift.setPower(0);
            }

            else if(currentLiftMode == LiftMode.DOWN && lift.getCurrentPosition() > 3.1){
                lift.setPower(0);
            }


            /** Code To Run The Arm In Tele-Op **/
            if(gamepad2.b && currentArmPosition != ArmPosition.HOME){
                currentArmPosition = ArmPosition.HOME;
                setArmServo(.25);
            }
            else if(gamepad2.x && currentArmPosition !=ArmPosition.DROP){
                currentArmPosition = ArmPosition.DROP;


            }
            else if(gamepad2.y){
                currentArmPosition = ArmPosition.REACH;
                setArmServo(.65);
            }

            if(Math.abs(gamepad2.right_stick_y)>.1){
                arm.setElbowPower(.5*gamepad2.right_stick_y);
            }
            else{
                moveArmTeleOp(currentArmPosition);
            }

            if(gamepad2.right_bumper){
                setArmServo(arm.getservoPos()+.05);

            }
            else if(gamepad2.left_bumper){
                setArmServo(arm.getservoPos()-.05);
            }
            if(arm.shoulderPosition()>1.0 && currentArmPosition == ArmPosition.DROP && arm.shoulderPosition()<2.0){
                setArmServo(0);
            }
            telemetry.addData("bucket pos", arm.getservoPos());


            telemetry.addData("liftPosition", lift.getCurrentPosition());
            telemetry.update();
        }
    }
}