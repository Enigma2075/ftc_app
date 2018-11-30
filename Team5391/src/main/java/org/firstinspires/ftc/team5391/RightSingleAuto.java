package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Right Single", group = "Main")
//@Disabled
public class RightSingleAuto extends LeftSingleAuto {

    @Override
    public void runOpMode() {
        turn = false;
        moveToCrater = false;

        super.runOpMode();

        // At this point we should be right next to the wall in the same spot regardless.
        gyroTurn(.8,35, TurnType.BOTH, true);

        gyroDrive(55, 35);

        slowIntake();
        sleep(300);

        gyroDrive(-62, 55);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        pivotThread.interrupt();
    }
}

