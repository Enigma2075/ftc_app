package org.firstinspires.ftc.team5391;

public class DriveThread extends Thread {
    private Teleop opMode;

    DriveThread(Teleop opMode) {
        this.opMode = opMode;
    }

    @Override
    public void run() {
        opMode.waitForStart();

        while(!interrupted() && !opMode.isStopRequested() && opMode.isStarted()) {
            opMode.drive();
            try {
                sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}