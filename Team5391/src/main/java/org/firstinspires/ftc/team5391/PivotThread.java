package org.firstinspires.ftc.team5391;

public class PivotThread extends Thread {
    private BaseOpMode opMode;

    PivotThread(BaseOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void run() {
        opMode.waitForStart();

        while(!interrupted() && !opMode.isStopRequested() && opMode.isStarted()) {
            opMode.updateIntakePivot(true);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}