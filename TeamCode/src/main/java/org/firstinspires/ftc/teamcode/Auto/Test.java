package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name = "TestAny", group = "godlyTest")
public class Test extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            boolean red = false;
            super.TestAny();
            break;
        }
    }
}
