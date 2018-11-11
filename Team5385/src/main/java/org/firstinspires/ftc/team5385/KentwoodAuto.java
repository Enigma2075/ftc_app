package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="kentwoodAuto", group="Pushbot")

public class KentwoodAuto extends BigAutoBase {

    public void runOpMode() {
        super.runOpMode();

        drivetrain.init(hardwareMap);
        gyro = hardwareMap.get(AdafruitBNO055IMU.class, "gyro");
        lift.init(hardwareMap);
        colorSystem.init(hardwareMap);
        arm.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(parameters);
        setArmServo(.3);

        while(!opModeIsActive()) {
            if(isStopRequested()) {
                return;
            }
            telemetry.addData("Gyro", gyro.getAngularOrientation().firstAngle);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive() && !gyro.isGyroCalibrated()) {
            if(isStopRequested()) {
                return;
            }
            sleep(50);
            idle();
        }

        moveLift(0.2);
        gyroDrive(.9,10,0);
        if(false) {
            gyroTurn(.9, -120);
            gyroDrive(.9, -12, -120);
            moveArm(ArmPosition.DROP);
            setArmServo(0);
            sleep(3000);
            moveArm(ArmPosition.HOME);
            gyroDrive(.9, 6, -100);
        }

    }


}
