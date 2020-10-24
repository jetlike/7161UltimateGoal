package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name="FoundationAutoRedMid", group = "godlyFoundation")
public class FoundationAutoRedMid extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            boolean red = true;
            boolean wall = false;
            super.FoundationAutoV2(red, wall);
            break;
        }
    }
}
