package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Single Block Auto Crater Side", group="Pushbot")
public class SingleBlockAutoCraterSide extends BigAutoBase{


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
        gyroDrive(.9, 17, -35);
        gyroTurn(.9,-100);
        gyroDrive(.9, 27, -100);
        gyroTurn(.9, -120, TurnType.RIGHT_ONLY);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroDrive(.9,-9,-120);
        gyroTurn(.9,-145, TurnType.LEFT_ONLY);
        gyroDrive(.9,-20,-145);

    }
    private void middleBlock(){
        gyroDrive(.9, 17, 0);
        gyroDrive(.9,-7,0);
        gyroTurn(.9, -85);
        gyroDrive(.9, 37, -85);
        gyroTurn(.9, -120, TurnType.RIGHT_ONLY);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroDrive(.9,-9,-120);
        gyroTurn(.9,-145, TurnType.LEFT_ONLY);
        gyroDrive(.9,-16,-145);
    }
    private void rightBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, 35);
        gyroDrive(.9, 17, 35);
        gyroDrive(.9,-8,35);
        gyroTurn(.9,-83);
        gyroDrive(.9,38,-83);
        gyroTurn(.9,-120, TurnType.RIGHT_ONLY);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
    }




}
