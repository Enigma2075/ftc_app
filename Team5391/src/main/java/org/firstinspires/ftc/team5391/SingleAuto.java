/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the drivetrain.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the drivetrain must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the drivetrain is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the drivetrain Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Single Auto", group = "Main")
//@Disabled
public class SingleAuto extends BaseOpMode {
    protected boolean rightBlock = false;
    protected boolean leftBlock = false;
    protected boolean centerBlock = false;
    protected boolean leftBall = false;
    protected boolean centerBall = false;

    protected boolean moveToCrater = true;

    @Override
    public void runOpMode() {
        super.runOpMode();

        // are landing
        movePower(.013);
        moveLift(8);
        movePower(0);

        //Pull away from lander
        gyroDrive(13.25, 0.0);
        moveLift(.5, true);
        rightKnockerCheck();

        //Turn parallel to the block and balls
        gyroTurn(-90);
        //enything up needs to be unslashed

        // check if middle is a block
        CheckForBlock check = new CheckForBlock();
        gyroDrive(DRIVE_SPEED * .2, 6, -90, check);

        if (check.foundBlock()) {
            // The middle is a block
            centerBlock = true;
            rightKnockerKnock();
            sleep(100);
            gyroDrive(-6, -90);
            rightKnockerUp();

            gyroDrive(46, -90);
        }   // dont need to change the line above this slash
        else {
            // Check if middle is ball
            centerBall = check.foundBall();

            // checks if the left is block
            gyroDrive(9.75, -90);
            check = new CheckForBlock();
            gyroDrive(DRIVE_SPEED * .2, 6, -90, check);
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
                if (leftBall && centerBall) {
                    rightBlock = true;
                    rightKnockerKnock();
                    sleep(100);
                    gyroDrive(-6, -90);
                    rightKnockerUp();
                    gyroDrive(47, -90);
                } else { //dont need to change the line ubove this slash
                    check = new CheckForBlock();
                    gyroDrive(DRIVE_SPEED * .2, -6, -90, check);
                    // Check right mineral
                    if (check.foundBlock()) {
                        rightBlock = true;
                        rightKnockerKnock();
                        sleep(100);
                        gyroDrive(6, -90);
                        rightKnockerUp();
                        gyroDrive(47, -90);
                    } //dont need to change the line ubove this slash
                    // If above aren't ball, hit right mineral
                    else {
                        //rightKnockerUp();
                        gyroDrive(58, -90);
                    } //dont need to change the line ubove this slash
                }
            }
        }

        // At this point we should be right next to the wall in the same spot regardless.
        gyroTurn(-145, TurnType.RIGHT_ONLY);

        if(moveToCrater) {
            gyroDrive(43, -130);

            gyroDrive(-63, -140);
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
}

