
package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="testOpMode", group="Pushbot")

public class TestOpMode extends BigAutoBase {

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

        while(opModeIsActive()){
            if(gamepad1.right_bumper){
                colorSystem.setPosition(colorSystem.getPosition()+.01);
            }
            else if(gamepad1.left_bumper){
                colorSystem.setPosition(colorSystem.getPosition()-.01);
            }
            telemetry.addData("sensor value:", colorSystem.getColor());
            telemetry.addData("Servo Pos" , colorSystem.getPosition());
            telemetry.update();
        }
    }

}
