package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Left Double Auto", group = "Main")
//@Disabled
public class LeftDoubleAuto extends LeftSingleAuto {
    @Override
    public void runOpMode() {
        moveToCrater = false;

        super.runOpMode();

        if(rightBlock) {
            gyroTurn(-150);
            gyroDrive(8, -150);
            gyroTurn(-120);
            gyroDrive(24, -120);
            //gyroTurn(-130);
        }
        // Hit the corresponding mineral
        else if(centerBlock) {
            gyroDrive(23, -140);
            gyroTurn(-230);
            gyroDrive(17, -230);
            gyroDrive(-17, -230);
            gyroTurn(-130);
        }
        else if(leftBlock) {
            gyroDrive(30, -140);
            gyroTurn(-225);
            gyroDrive(27, -225);
            gyroDrive(-27, -225);
            gyroTurn(-140);
            //gyroDrive(-3, -140);
        }
        else {
            gyroDrive(43, -130);
        }

        slowIntake();
        sleep(300);

        if(rightBlock) {
            gyroDrive(-57, -140);
        }
        else if(centerBlock) {
            gyroDrive(-58, -140);
        }
        else if(leftBlock) {
            gyroDrive(-55, -140);
        }
        else {
            gyroDrive(-55, -140);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

        pivotThread.interrupt();
    }
}

