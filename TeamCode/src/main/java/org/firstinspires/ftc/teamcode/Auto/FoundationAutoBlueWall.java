package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name="FoundationAutoBlueWall", group = "godlyFoundation")
public class FoundationAutoBlueWall extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            boolean red = false;
            boolean wall = true;
            super.FoundationAutoV2(red, wall);
            break;
        }
    }
}