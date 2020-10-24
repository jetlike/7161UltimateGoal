package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name="FoundationAutoBlueMid", group = "godlyFoundation")
public class FoundationAutoBlueMid extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            boolean red = false;
            boolean wall = false;
            super.FoundationAutoV2(red, wall);
            break;
        }
    }
}
