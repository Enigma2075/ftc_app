package org.firstinspires.ftc.team5391;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BaseOpMode extends LinearOpMode {

    private HardwareDrivetrain drivetrain = new HardwareDrivetrain();
    private HardwareLift lift = new HardwareLift();

    private AdafruitBNO055IMU gyro = null;                    // Additional Gyro device
    private DistanceSensor sensorRange1;
    private DistanceSensor sensorRange2;
    private DistanceSensor sensorRange3;
    private Servo leftKnocker;
    private Servo rightKnocker;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific drivetrain drive train.
    static final double DRIVE_SPEED = .75;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.8;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = .15;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.065;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.02;     // Larger is more responsive, but also less stable

    static final double RIGHT_KNOCKER_UP = 0;
    static final double LEFT_KNOCKER_UP = 1;
    static final double RIGHT_KNOCKER_CHECK = .445;
    // need to update in future
    static final double RIGHT_KNOCKER_KNOCK = 0.6;
    static final double LEFT_KNOCKER_KNOCK = 0.4;

    static final double LIFT_BOTH_UP = 1.19;
    static final double LIFT_BOTH_DOWN = -1.19;


    public class CheckForBlock implements Runnable {
        double minDistance = 500;

        public boolean foundBlock() {
            return minDistance < 76 && !foundBall();
        }

        public boolean foundBall() {
            return minDistance < 25;
        }

        @Override
        public void run() {
            double distance1 = sensorRange1.getDistance(DistanceUnit.MM);
            double distance2 = sensorRange2.getDistance(DistanceUnit.MM);
            double distance3 = sensorRange3.getDistance(DistanceUnit.MM);
            if (distance1 < minDistance) {
                minDistance = distance1;
            }
            if (distance2 < minDistance) {
                minDistance = distance2;
            }
            if (distance3 < minDistance) {
                minDistance = distance3;
            }

            telemetry.addData("deviceName", sensorRange1.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", sensorRange1.getDistance(DistanceUnit.MM)));
            telemetry.addData("minDistance", String.format("%.01f mm", minDistance));
        }
    }

    @Override
    public void runOpMode() {
        sensorRange1 = hardwareMap.get(DistanceSensor.class, "sensor_range1");
        sensorRange2 = hardwareMap.get(DistanceSensor.class, "sensor_range2");
        sensorRange3 = hardwareMap.get(DistanceSensor.class, "sensor_range3");
        leftKnocker = hardwareMap.get(Servo.class, "range_servo");
        rightKnocker = hardwareMap.get(Servo.class, "range_servo2");

        gyro = hardwareMap.get(AdafruitBNO055IMU.class, "imu");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        leftKnockerUp();
        rightKnockerUp();

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        lift.init(hardwareMap);
        drivetrain.init(hardwareMap);

        initGyro();


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %f", gyro.getAngularOrientation().firstAngle);
            telemetry.update();
        }
    }

    protected void rightKnockerUp() {
        rightKnocker.setPosition(RIGHT_KNOCKER_UP);
    }

    protected void leftKnockerUp() {
        leftKnocker.setPosition(LEFT_KNOCKER_UP);
    }

    protected void rightKnockerCheck() {
        rightKnocker.setPosition(RIGHT_KNOCKER_CHECK);
    }

    protected void leftKnockerKnock() {
        leftKnocker.setPosition(LEFT_KNOCKER_KNOCK);
    }

    protected void rightKnockerKnock() {
        rightKnocker.setPosition(RIGHT_KNOCKER_KNOCK);
    }

    protected void initGyro() {
        // Ensure the drivetrain it stationary, then reset the encoders and calibrate the gyro.
        drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected boolean onHeading(double speed, double angle, double PCoeff) {
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
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        drivetrain.setPower(rightSpeed, leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    protected double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngularOrientation().firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    protected double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    protected void gyroTurn(double angle) {
        gyroTurn(TURN_SPEED, angle);
    }

    protected void gyroTurn(double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, -angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    protected void gyroDrive(double distance,
                             double angle) {
        gyroDrive(DRIVE_SPEED, distance, angle, null);
    }

    protected void gyroDrive(double speed,
                             double distance,
                             double angle) {
        gyroDrive(speed, distance, angle, null);
    }

    protected void gyroDrive(double speed,
                             double distance,
                             double angle, Runnable method) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            angle = -angle;
            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * drivetrain.COUNTS_PER_INCH);
            newLeftTarget = drivetrain.getLeftCurrentPosition() + moveCounts;
            newRightTarget = drivetrain.getRightCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            drivetrain.setTargetPosition(newRightTarget, newLeftTarget);

            drivetrain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            drivetrain.setPower(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (drivetrain.isLeftBusy() && drivetrain.isRightBusy())) {

                if (method != null) {
                    method.run();
                }

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

                drivetrain.setPower(rightSpeed, leftSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", drivetrain.getLeftCurrentPosition(),
                        drivetrain.getRightCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            drivetrain.setPower(0, 0);

            // Turn off RUN_TO_POSITION
            drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    protected void moveLift(double height) {
        moveLift(height, 1);
    }

    protected void moveLift(double height, double power) {
        lift.setTargetPosition(height);

        while (opModeIsActive() && lift.isBusy())
        {
            lift.setPower(power);
        }
    }
}