
package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
@Autonomous(name="autoBlocks", group="Pushbot")

public class AutoHitBlocks extends BigAutoBase {

    /* Declare OpMode members. */

    @Override
    public void runOpMode() {
        super.runOpMode();

        arm.init(hardwareMap);
        drivetrain.init(hardwareMap);
        gyro = hardwareMap.get(AdafruitBNO055IMU.class, "gyro");
        lift.init(hardwareMap);
        colorSystem.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(parameters);

        waitForStart();

        moveLift(3.275);

        moveArm(ArmPosition.HOME);

        setArmServo(.3);
    }

}
