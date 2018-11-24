package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "TeleOp drive", group= "learn TeleOp"  )
public class Teleop extends BaseOpMode {
    CheesyDrive cheesyDrive = null;

    DriveThread driveThread = null;

    @Override
    public void runOpMode() {
        isAuto = false;
        super.runOpMode();

        cheesyDrive = new CheesyDrive();
        driveThread = new DriveThread(this);

        waitForStart();

        driveThread.start();

        currentIntakePivot = HardwareIntake.IntakePivot.STORE;

        while (opModeIsActive()) {

            if(gamepad1.y) {
                // SCORE
                //setIntakePivotIfNotOverride(HardwareIntake.IntakePivot.BALLS);
                moveLift(7.0);
                //sleep(500);
                moveLift(8.1, .3);
                //setIntakeExtension(9);
                moveLift(.5, true);
                //sleep(500);
                //moveLift(.5);
            }
            //else if(gamepad1.y) {
            //    //Move up and disconnect
            //    moveLift(8.1);
            //}
            else if(gamepad1.a){
                //Move Down
                moveLift(.5);
            }
            else if(gamepad1.b) {
                //Pre-Latch
                moveLift(4.5);
            }
            else if(gamepad1.x) {
                //Latch
                moveLift(7.25);
            }

            if (gamepad2.right_bumper && !isInCrater()) {
                if(gamepad2.left_trigger > .5) {
                    spitIntake();
                }
                else {
                    suckIntake();
                }
                collectInCrater();
            }
            else if (gamepad2.right_bumper && Math.abs(gamepad2.left_stick_y) > 0) {
                if(gamepad2.left_trigger > .5) {
                    spitIntake();
                }
                else {
                    suckIntake();
                }
                setIntakePivotIfNotOverride(HardwareIntake.IntakePivot.STORE);
                moveIntakeExtension(-gamepad2.left_stick_y);
            }
            else if(gamepad2.right_bumper){
                if(gamepad2.left_trigger > .5) {
                    spitIntake();
                }
                else {
                    suckIntake();
                }
                setIntakePivotIfNotOverride(HardwareIntake.IntakePivot.STORE);
                setIntakeExtension();
            }
            //else if(gamepad2.left_bumper){
            //    currentIntakePivot = HardwareIntake.IntakePivot.TRANSFER;
            //    setIntakeExtension(.5, true);
            //    slowIntake();
            //}
            else {
                if(getLiftHeight() > 1) {
                    currentIntakePivot = HardwareIntake.IntakePivot.STORE;
                    setIntakeExtension(6, true);
                    intakeOff();
                }
                else {
                    if(getIntakeExtension() < 3) {
                        currentIntakePivot = HardwareIntake.IntakePivot.TRANSFER;
                    }
                    else {
                        currentIntakePivot = HardwareIntake.IntakePivot.STORE;
                    }
                    setIntakeExtension(.5, true);
                    slowIntake();
                }
            }

            if(gamepad2.a) {
                currentIntakePivot = HardwareIntake.IntakePivot.BALLS;
            }
            else if(gamepad2.b) {
                currentIntakePivot = HardwareIntake.IntakePivot.BLOCKS;
            }
            else if(gamepad2.y) {
                currentIntakePivot = HardwareIntake.IntakePivot.STORE;
            }
            else if(gamepad2.x) {
                currentIntakePivot = HardwareIntake.IntakePivot.TRANSFER;
            }

            sendTelemetry();
            telemetry.update();
        }

        driveThread.interrupt();
        pivotThread.interrupt();
    }

    public void drive() {
        //moves lift and drivetrain
        boolean quickTurn = false;
        if(Math.abs(gamepad1.left_stick_y) == 0) {
            quickTurn = true;
        }

        double throttle = gamepad1.left_stick_y * -1;
        double steer = gamepad1.right_stick_x;

        if(getIntakeExtension() > 12) {
            steer *= .5;
        }

        DriveSignal powers = cheesyDrive.cheesyDrive(throttle, steer, quickTurn);
        drive(powers);
    }

    private boolean driverOverride() {
        return gamepad2.a || gamepad2.b ||gamepad2.x || gamepad2.y;
    }

    private void setIntakePivotIfNotOverride(HardwareIntake.IntakePivot pivot) {
        if(!driverOverride()) {
            currentIntakePivot = pivot;
        }
    }
}