package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "TeleOp drive", group= "learn TeleOp"  )
public class Teleop extends BaseOpMode {
    CheesyDrive cheesyDrive = null;

    @Override
    public void runOpMode() {
        isAuto = false;
        super.runOpMode();

        cheesyDrive = new CheesyDrive();
        waitForStart();

        currentIntakePivot = HardwareIntake.IntakePivot.STORE;

        while (opModeIsActive()) {
            //moves lift and drivetrain
            boolean quickTurn = false;
            if(Math.abs(gamepad1.left_stick_y) == 0) {
                quickTurn = true;
            }

            DriveSignal powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, quickTurn);
            drive(powers);

            //goes up with operator y/yellow
            if(gamepad2.y) {
                moveLift(8.1);
            }
            else if(gamepad2.a){
                moveLift(.5);
            }
            else if(gamepad2.b) {
                moveLift(5.5);
            }
            else if(gamepad2.x) {
                moveLift(7.25);
            }

            powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, quickTurn);
            drive(powers);

            if (gamepad2.right_bumper && !isInCrater()) {
                collectInCrater();
            }
            else if (gamepad2.right_bumper && Math.abs(gamepad2.right_stick_y) > 0) {
                suckIntake();
                setIntakePivotIfNotOverride(HardwareIntake.IntakePivot.BALLS);
                moveIntakeExtension(-gamepad2.right_stick_y);
            }
            else if(gamepad2.right_bumper){
                setIntakePivotIfNotOverride(HardwareIntake.IntakePivot.BALLS);
                setIntakeExtension();
            }
            else if(gamepad2.left_bumper){
                currentIntakePivot = HardwareIntake.IntakePivot.TRANSFER;
                setIntakeExtension(.5, true);
                slowIntake();
            }
            else if(gamepad2.dpad_up) {
                setIntakePivotIfNotOverride(HardwareIntake.IntakePivot.BALLS);
                moveLift(6.5);
                sleep(500);
                moveLift(8.1);
                setIntakeExtension(9);
                moveLift(3.5);
                sleep(500);
                moveLift(.5);
            }
            else {
                if(getLiftHeight() > 1) {
                    currentIntakePivot = HardwareIntake.IntakePivot.STORE;
                    setIntakeExtension(6);
                    intakeOff();
                }
                else {
                    currentIntakePivot = HardwareIntake.IntakePivot.STORE;
                    setIntakeExtension(1.5);
                    intakeOff();
                }
            }

            powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, quickTurn);
            drive(powers);

            if(gamepad1.a) {
                currentIntakePivot = HardwareIntake.IntakePivot.BALLS;
            }
            else if(gamepad1.b) {
                currentIntakePivot = HardwareIntake.IntakePivot.BLOCKS;
            }
            else if(gamepad1.y) {
                currentIntakePivot = HardwareIntake.IntakePivot.STORE;
            }
            else if(gamepad1.x) {
                currentIntakePivot = HardwareIntake.IntakePivot.TRANSFER;
            }

            powers = cheesyDrive.cheesyDrive(gamepad1.left_stick_y * -1, gamepad1.right_stick_x, quickTurn);
            drive(powers);

            //updateIntakePivot(true);

            sendTelemetry();
            telemetry.update();
            keepAlive.interrupt();
        }
    }

    private boolean driverOverride() {
        return gamepad1.a || gamepad1.b ||gamepad1.x || gamepad1.y;
    }

    private void setIntakePivotIfNotOverride(HardwareIntake.IntakePivot pivot) {
        if(!driverOverride()) {
            currentIntakePivot = pivot;
        }
    }
}