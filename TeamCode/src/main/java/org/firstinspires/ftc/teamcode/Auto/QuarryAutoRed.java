package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name="QuarryAutoRed", group = "godlyQuarry")
public class QuarryAutoRed extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            boolean red = true;
            super.QuarryAutoV2(red);
            break;
        }
    }
}
