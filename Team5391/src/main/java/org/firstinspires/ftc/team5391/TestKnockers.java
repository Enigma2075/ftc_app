
package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestKnockers", group = "Test")
//@Disabled
public class TestKnockers extends BaseOpMode {
    @Override
    public void runOpMode() {
        super.runOpMode();

        rightKnockerCheck();

        sleep(4000);

        CheckForBlock check = new CheckForBlock();

        while(opModeIsActive()) {
            check.run();
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
