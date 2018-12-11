package org.firstinspires.ftc.team5385;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.sun.tools.javac.tree.JCTree;

@Autonomous(name="Double Block Auto Crater Side", group="Pushbot")

public class DoubleBlockAutoCraterSide extends BigAutoBase {


    @Override
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
        arm.setServoPos(0);
        moveLift(0.2);
        boolean blue = checkSensorAt(.327);
        telemetry.addData("blueLight", blue);
        telemetry.update();
        if(blue){
            colorSystem.setPosition(1);
            middleBlock();
        }
        else{
            blue = checkSensorAt(.5);
            if(blue){
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

    public void rightBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, 35);
        gyroDrive(.9, 17, 35);
        gyroDrive(.9,-8,35);
        gyroTurn(.9,-83);
        gyroDrive(.9,36,-83);
        gyroTurn(.9,-140, TurnType.RIGHT_ONLY);
        gyroDrive(.9,8,-140);
        gyroTurn(.9,-120);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroTurn(.9,-142);
        gyroDrive(.9,-32,-142);

    }
    public void leftBlock(){
        gyroDrive(.9, 3, 0);
        gyroTurn(.9, -35);
        gyroDrive(.9, 17, -35);
        gyroTurn(.9,-100);
        gyroDrive(.9, 30, -100);
        gyroTurn(.9, -120, TurnType.RIGHT_ONLY);
        gyroDrive(.9, 20, -120);
        gyroTurn(.9, -215, TurnType.RIGHT_ONLY);
        gyroDrive(.9, 15, -215);
        gyroDrive(.9, -17, -215);
        gyroTurn(.9, -135, TurnType.RIGHT_ONLY);
        gyroDrive(.9, -18, -135);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.HOME);
        gyroDrive(.9, -67, -135);
    }
    public void middleBlock(){
        gyroDrive(.9, 19, 0);
        gyroDrive(.9,-5,0);
        gyroTurn(.9, -85);
        gyroDrive(.9, 35, -85);
        gyroTurn(.9, -120, TurnType.RIGHT_ONLY);
        //possible Marker Drop
        gyroDrive(.9,17,-120);
        gyroTurn(.9,-222, TurnType.RIGHT_ONLY);
        gyroDrive(.9,5,-222);
        gyroDrive(.9,-5,-222);
        gyroTurn(.9,-130,TurnType.RIGHT_ONLY);
        gyroDrive(.9,-12,-130);
        moveArm(ArmPosition.MARKER);
        moveArm(ArmPosition.PARK);
        gyroDrive(.9,-36,-130);
        /*gyroDrive(.9,4,-222);
        gyroDrive(.9,-4,-222);
        gyroTurn(.9,-131, TurnType.RIGHT_ONLY);
        gyroDrive(.9, -12, -131);
        gyroDrive(.9,-50,-131);
        */
    }

}
