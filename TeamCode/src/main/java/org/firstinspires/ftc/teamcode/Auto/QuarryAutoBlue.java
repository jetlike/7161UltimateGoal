package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name = "QuarryAutoBlue", group = "godlyQuarry")
public class QuarryAutoBlue extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            boolean red = false;
            super.QuarryAutoV2(red);
            break;
        }
    }
}
