package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Single", group = "Main")
//@Disabled
public class LeftSingleAuto extends BaseOpMode {
    protected boolean rightBlock = false;
    protected boolean leftBlock = false;
    protected boolean centerBlock = false;
    protected boolean leftBall = false;
    protected boolean centerBall = false;

    protected boolean moveToCrater = true;

    protected boolean turn = true;

    @Override
    public void runOpMode() {
        super.runOpMode();

        // are landing
        currentIntakePivot = HardwareIntake.IntakePivot.STORE;
        //moveLift(.5, true);
        movePower(.013);
        setIntakeExtension(6.5, true);
        moveLift(8.325);
        movePower(0);

        //Pull away from lander
        gyroDrive(13.25, 0.0);
        moveLift(.5, true);
        rightKnockerCheck();

        //Turn parallel to the block and balls
        gyroTurn(-90);

        currentIntakePivot = HardwareIntake.IntakePivot.IN_DUMP;
        //updateIntakePivot(false);
        setIntakeExtension(2, true);
        //updateIntakePivot(true);

        // check if middle is a block
        CheckForBlock check = new CheckForBlock();
        //updateIntakePivot(true);
        gyroDrive(.1, 7, -90, check);

        if (check.foundBlock()) {
            // The middle is a block
            centerBlock = true;
            rightKnockerKnock();
            sleep(100);
            gyroDrive(-7, -90);
            rightKnockerUp();

            gyroDrive(48, -90);
        }   // dont need to change the line above this slash
        else {
            // Check if middle is ball
            centerBall = check.foundBall();

            // checks if the left is block
            gyroDrive(9.25, -90);
            check = new CheckForBlock();
            gyroDrive(.1, 6, -90, check);
            if (check.foundBlock()) {
                // The left is a block
                leftBlock = true;
                rightKnockerKnock();
                sleep(100);
                gyroDrive(-6, -90);
                rightKnockerUp();
                gyroDrive(31, -90);
            }    //dont need to change the line ubove this slash
            else {
                leftBall = check.foundBall();
                gyroDrive(-27, -90);

                //  If ball, continue
                rightBlock = true;
                rightKnockerKnock();
                sleep(100);
                gyroDrive(-6, -90);
                rightKnockerUp();
                if(moveToCrater) {
                    gyroDrive(58, -90);
                }
                else {
                    gyroDrive(47, -90);
                }
            }
        }

        if(turn) {
            // At this point we should be right next to the wall in the same spot regardless.
            gyroTurn(.8, -130, TurnType.RIGHT_ONLY);
        }
        else {
        //    gyroDrive(-6, -90);
        }

        if(moveToCrater && turn) {
            gyroDrive(30, -130);

            slowIntake();
            sleep(300);

            gyroDrive(-65, -135);
            telemetry.addData("Path", "Complete");
            telemetry.update();
            pivotThread.interrupt();
        }
    }
}

