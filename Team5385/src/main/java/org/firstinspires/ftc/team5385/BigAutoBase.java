package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

public class BigAutoBase  extends LinearOpMode {

    HardwareDrivetrain drivetrain = new HardwareDrivetrain();   // Use a Pushbot's hardware
    HardwareLift lift = new HardwareLift();
    HardwareColorSensor colorSystem = new HardwareColorSensor();
    AdafruitBNO055IMU gyro = null;// Additional Gyro device


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .75;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 1;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.7;     // Nominal half speed for better accuracy

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.02;     // Larger is more responsive, but also less stable
    static final double P_MOVE_LIFT_COEFF = 6;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 1.0/20.0;

    @Override
    public void runOpMode() {

    }



    public void gyroDrive(double speed, double distance, double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        angle = -angle;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = drivetrain.getCurrentLeftPosition() + moveCounts;
            newRightTarget = drivetrain.getCurrentRightPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            drivetrain.setTarget(newLeftTarget, newRightTarget);
            drivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            drivetrain.setPower(speed, speed);

            int notBusyCount = 0;
            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    notBusyCount < 2) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                drivetrain.setPower(leftSpeed, rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", drivetrain.getCurrentLeftPosition(),
                        drivetrain.getCurrentRightPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();

                if(drivetrain.isLeftBusy() && drivetrain.isRightBusy()) {
                    notBusyCount = 0;
                }
                else {
                    notBusyCount++;
                }
            }

            // Stop all motion;
            drivetrain.setPower(0, 0);

            // Turn off RUN_TO_POSITION
            drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (

        robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroHold(double speed, double angle, double holdTime) {
        angle = -angle;
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF, TurnType.BOTH);
            telemetry.update();
        }

        // Stop all motion;
        drivetrain.setPower(0, 0);
    }

    public enum TurnType {BOTH, RIGHT_ONLY, LEFT_ONLY}

    public void gyroTurn(double speed, double angle) {
        gyroTurn(speed, angle, TurnType.BOTH);
    }

    public void gyroTurn(double speed, double angle, TurnType turnType) {
        angle = -angle;
        // keep looping while we are still active, and not on heading.

        int onHeadingCount = 0;
        while (opModeIsActive() && onHeadingCount < 2) {

            if(!onHeading(speed, angle, P_TURN_COEFF, turnType)) {
               onHeadingCount = 0;
            }
            else {
                onHeadingCount++;
            }

            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff, TurnType turnType) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);

            if(turnType == TurnType.BOTH || turnType == TurnType.RIGHT_ONLY) {
                rightSpeed = speed * steer;
            }
            else {
                rightSpeed = 0;
            }

            if(turnType == TurnType.BOTH || turnType == TurnType.LEFT_ONLY) {
                leftSpeed = -speed * steer;
            }
            else {
                leftSpeed = 0;
            }
        }

        // Send desired speeds to motors.
        drivetrain.setPower(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    protected void moveLift(double target) {
        while (Math.abs(lift.getError(target)) > .04 && opModeIsActive()) {
            double power = lift.getError(target) * P_MOVE_LIFT_COEFF;
            lift.setPower(power);
            telemetry.addData("servoPower", lift.getPosition());
            telemetry.addData("Power", power);
            telemetry.addData("Voltage", lift.getCurrentPosition());
            telemetry.addData("Error", lift.getError(target));
            telemetry.update();
        }
        lift.setPower(0);

    }

    public double checkSensorAt(double startPosition){
        double highestSensorValue= 0;
        for(double i = startPosition; i< startPosition +.1; i+=.005){
            colorSystem.setPosition(i);
            sleep(175);
            if(colorSystem.getColor()>highestSensorValue) highestSensorValue = colorSystem.getColor();
        }
        return highestSensorValue;

    }
}