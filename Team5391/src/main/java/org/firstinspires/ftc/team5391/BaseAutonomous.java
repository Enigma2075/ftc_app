package org.firstinspires.ftc.team5391;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class BaseAutonomous extends LinearOpMode {

    HardwareDrivetrain drivetrain = new HardwareDrivetrain();   // Use a Pushbot's hardware
    AdafruitBNO055IMU gyro = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 537.6  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific drivetrain drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.02;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            angle = -angle;
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = drivetrain.getLeftCurrentPosition() + moveCounts;
            newRightTarget = drivetrain.getRightCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            drivetrain.setTargetPosition(newRightTarget, newLeftTarget);
            /*
            drivetrain.leftDrive.setTargetPosition(newLeftTarget);
            drivetrain.rightDrive.setTargetPosition(newRightTarget);
            drivetrain.leftDrive2.setTargetPosition(newLeftTarget);
            drivetrain.rightDrive2.setTargetPosition(newRightTarget);
            */

            drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            drivetrain.setPower(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (drivetrain.isLeftBusy() && drivetrain.isRightBusy())) {

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
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                drivetrain.setPower(rightSpeed, leftSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      drivetrain.getLeftCurrentPosition(),
                        drivetrain.getRightCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            drivetrain.setPower(0, 0);

            // Turn off RUN_TO_POSITION
            drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
