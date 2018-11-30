
package org.firstinspires.ftc.team5391;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Reset", group = "Main")
//@Disabled
public class Reset extends BaseOpMode {
    @Override
    public void runOpMode() {
        super.runOpMode();

        resetEncoders();
    }
}

