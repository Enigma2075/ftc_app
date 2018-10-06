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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single drivetrain.
 * In this case that drivetrain is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the drivetrain:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareDrivetrain
{
    /* Public OpMode members. */
    private DcMotor  leftDrive   = null;
    private DcMotor  rightDrive  = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDrivetrain(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "LeftDrive");
        rightDrive = hwMap.get(DcMotor.class, "RightDrive");
       leftDrive2 = hwMap.get (DcMotor.class, "LeftDrive2");
       rightDrive2 = hwMap.get(DcMotor.class, "RightDrive2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        setPower(0, 0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
  // Mode
    public void setRightMode(DcMotor.RunMode mode) {
        rightDrive.setMode(mode);
        rightDrive2.setMode(mode);
    }

    public void setLeftMode(DcMotor.RunMode mode){
        leftDrive.setMode(mode);
        leftDrive2.setMode(mode);
    }

    public void setMode(DcMotor.RunMode rightMode, DcMotor.RunMode leftMode){
        setLeftMode(leftMode);
        setRightMode(rightMode);
    }

    public void setMode(DcMotor.RunMode mode) {
        setLeftMode(mode);
        setRightMode(mode);
    }
  // Power
    public void setRightPower(double power) {
        rightDrive.setPower(power);
        rightDrive2.setPower(power);
    }

    public void setLeftPower(double power) {
        leftDrive.setPower(power);
        leftDrive2.setPower(power);
    }

    public void setPower(double rightPower, double leftPower) {
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setPower(double power){
        setRightPower(power);
        setLeftPower(power);
    }
  // Target Position
    public void setRightTargetPosition(int targetPosition) {
        rightDrive.setTargetPosition(targetPosition);
        rightDrive2.setTargetPosition(targetPosition);
    }

    public void setLeftTargetPosition(int targetPosition){
        leftDrive.setTargetPosition(targetPosition);
        leftDrive2.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(int rightTargetPosition, int leftTargetPosition){
        setLeftTargetPosition(leftTargetPosition);
        setRightTargetPosition(rightTargetPosition);
    }

    public void setTargetPosition(int targetPosition) {
        setLeftTargetPosition(targetPosition);
        setRightTargetPosition(targetPosition);
    }
  // Current Position
     public int getRightCurrentPosition(){
        return rightDrive.getCurrentPosition();
     }

     public int getLeftCurrentPosition() {
         return leftDrive.getCurrentPosition();
     }
  // Is Busy
     public boolean isRightBusy(){
        return rightDrive.isBusy();
     }

     public boolean isLeftBusy(){
        return leftDrive.isBusy();
     }
 }

