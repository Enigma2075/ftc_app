package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Single Block Auto Depot Side", group="Pushbot")

public class SingleAutoDepotSize extends BigAutoBase{



    @Override
    public void runOpMode(){
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
        arm.setServoPos(0);
        moveLift(0.2);
        boolean isBlue = checkSensorAt(.327);
        telemetry.addData("blueLight", isBlue);
        telemetry.update();
        if(isBlue){
            colorSystem.setPosition(1);
            middleBlock();
        }
        else{
            isBlue = checkSensorAt(.5);
            if(isBlue){
                colorSystem.setPosition(1);
                leftBlock();
            }
            else{
                colorSystem.setPosition(1);
                rightBlock();
            }
        }

        moveLift(1.2);



        while(opModeIsActive()){

        }
        drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void leftBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, -35);
        gyroDrive(.9, 27, -35);
        gyroTurn(.9,20);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroTurn(.9,64);
        gyroDrive(.75,-34,64);
    }
    private void middleBlock(){
        gyroDrive(.9, 20, 0);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroDrive(.9,-5,0);
        gyroTurn(.9,90);
        gyroDrive(.9,-38,90);
        gyroTurn(.9,60);
        gyroDrive(.75,-14,60);

    }
    private void rightBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, 35);
        gyroDrive(.9, 20, 35);
        gyroTurn(.9,90,TurnType.RIGHT_ONLY);
        gyroDrive(.9,-48,90);
        gyroTurn(.9,40,TurnType.LEFT_ONLY);
        gyroDrive(.9,24,40);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroDrive(.75,-30,40);



    }



}
