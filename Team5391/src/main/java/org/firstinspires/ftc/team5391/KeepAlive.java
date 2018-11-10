package org.firstinspires.ftc.team5391;

public class KeepAlive extends Thread {
    private BaseOpMode opMode;

    KeepAlive(BaseOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void run() {
        while(!interrupted()) {
            opMode.updateIntakePivot(true);
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}