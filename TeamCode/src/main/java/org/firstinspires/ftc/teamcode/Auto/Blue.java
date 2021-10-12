package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous(name = "BlueAuto", group = "7161Blue")
public class Blue extends AutoMaster {


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            super.BlueAuto();
            break;
        }
    }
}
