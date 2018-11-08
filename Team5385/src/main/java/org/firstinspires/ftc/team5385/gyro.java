package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;

@Autonomous(name="gyroMove", group="Pushbot")

public class gyro extends BigAutoBase {


    @Override
    public void runOpMode() {
        super.runOpMode();

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

        moveLift(0.2);
        double blue = checkSensorAt(.327);
        telemetry.addData("blueLight", blue);
        telemetry.update();
        if(blue <= 2){
                middleBlock();
        }
        else{
            blue = checkSensorAt(.5);
            if(blue <= 2){
                leftBlock();
            }
            else{
                rightBlock();
            }
        }

        moveLift(3.275);

        while(opModeIsActive()){

        }
        drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rightBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, 35);
        gyroDrive(.9, 17, 35);
        gyroDrive(.9,-11,35);
        gyroTurn(.9,-70);
        gyroDrive(.9,24,-70);
        gyroTurn(.9,-120, TurnType.RIGHT_ONLY);
        gyroDrive(.9, 36, -120);
        gyroTurn(.9, -135);
        gyroDrive(.9, -55, -135);
    }
    public void leftBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, -35);
        gyroDrive(.9, 17, -35);
        gyroTurn(.9,-90);
        gyroDrive(.9, 21.5, -90);
        gyroTurn(.9, -130, TurnType.RIGHT_ONLY);
        gyroDrive(.9, 27, -130);
        gyroTurn(.9, -215, TurnType.RIGHT_ONLY);
        gyroDrive(.9, 15, -215);
        gyroDrive(.9, -19, -215);
        gyroTurn(.9, -135, TurnType.RIGHT_ONLY);
        gyroDrive(.9, -67, -130);
    }
    public void middleBlock(){
        gyroDrive(.9, 18, 0);
        gyroDrive(.9,-10,0);
        gyroTurn(.9, -72);
        gyroDrive(.9, 32, -72);
        gyroTurn(.9, -122);
        gyroDrive(.9, 27, -122);
        gyroTurn(.9,-222, TurnType.RIGHT_ONLY);
        gyroDrive(.9,4,-222);
        gyroDrive(.9,-4,-222);
        gyroTurn(.9,-131, TurnType.RIGHT_ONLY);
        gyroDrive(.9,-50,-131);
    }

}
