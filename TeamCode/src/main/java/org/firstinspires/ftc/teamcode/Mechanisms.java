package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Mechanisms {
    private AutoMaster auto = null;
    Servo found;
    Servo found2;
    Servo clamp;
    Servo capstone;
    TouchSensor touch1;
    TouchSensor touch2;

    public void Init(AutoMaster autoMaster) {
        auto = autoMaster;
        found = auto.hardwareMap.servo.get("found1"); //found1 is on the right
        found2 = auto.hardwareMap.servo.get("found2"); //found2 is on the left
        clamp = auto.hardwareMap.servo.get("clamp");
        capstone = auto.hardwareMap.servo.get("capstone");
        touch1 = auto.hardwareMap.touchSensor.get("touch1");
        touch2 = auto.hardwareMap.touchSensor.get("touch2");
        found.setPosition(0);
        found2.setPosition(1);
        clamp.setPosition(0);
        capstone.setPosition(1);

        // init servo
    }

    public void Update(boolean grab) {
        // Grab amount
    }

    public void GrabBrick(double targetpos) {
        auto.runtime.reset();
        while (auto.runtime.milliseconds() < 400 && auto.opModeIsActive()) {
            clamp.setPosition(targetpos);
        }
    }

    public void Foundation(double found1val, double found2val) {

        found.setPosition(found1val);
        found2.setPosition(found2val);


    }
}
