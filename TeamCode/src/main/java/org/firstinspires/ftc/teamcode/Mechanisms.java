package org.firstinspires.ftc.teamcode;

import android.media.audiofx.AutomaticGainControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Movement;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.sql.ResultSet;



public class Mechanisms {
    private AutoMaster auto = null;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    DcMotor intake;
    DcMotor arm;
    Servo lifter;
    Servo flicker;
    Servo wobbler;
    Servo angler;
    double armPos = 310;

    double motorVelocity = -2500;


    public void Init(AutoMaster autoMaster) {
        auto = autoMaster;
        shoot1 = auto.hardwareMap.get(DcMotorEx.class,"s1");
        shoot2 = auto.hardwareMap.get(DcMotorEx.class,"s2");
        intake = auto.hardwareMap.dcMotor.get("intake");
        lifter = auto.hardwareMap.servo.get("lifter");
        flicker = auto.hardwareMap.servo.get("flicker");
        wobbler = auto.hardwareMap.servo.get("wobbler");
        angler = auto.hardwareMap.servo.get("angler");
        arm = auto.hardwareMap.dcMotor.get("arm");
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbler.setPosition(.85);
        angler.setPosition(0);
        lifter.setPosition(.9);
        flicker.setPosition(1);

        // init servo
    }

    public void Update(boolean grab) {
        // Grab amount
    }


    public void powerShot() {
        startIntake();
        angler.setPosition(auto.anglerPower);
        auto.sleep(200);
        lifter.setPosition(auto.lifterUp);
        stopIntake();
        startShoot(auto.mechanisms.getNeededPower(auto.voltage));
        auto.sleep(1500);
        shoot();
        auto.sleep(200);
        stopShoot();
        lifter.setPosition(auto.lifterDown);
    }

    public void flicktime() {
        auto.sleep(140 );
        flicker.setPosition(.60);
        auto.sleep(100);
        flicker.setPosition(1);

    }

    public void shoot() {
        for (int i = 0; i<3; i++) {
            flicktime();
            if (i==2)
                break;
            auto.movement.rightGyroStrafe(1, 6, 1.5, 180);
            if (!(i==1)) {
                auto.sleep(200);
            } else {
                auto.sleep(300);
            }

              //  auto.movement.turnPD(173 + (i+1) * 4, 1, .6, 3);
            }
        }

    public void startIntake() {
        intake.setPower(.85);
    }

    public void stopIntake() { intake.setPower(0); }

    public void armWobble(boolean grabbed, boolean park, boolean second) {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        while (!auto.isStopRequested() && auto.opModeIsActive()) {
            while (arm.getCurrentPosition() < armPos) {
                if (second) {
                    auto.movement.resetOdomEncoders();
                while (Math.abs(auto.movement.verticalOdomAverage()) < 150) {
                    auto.movement.startMotors(-.1, -.1);
                    }
                }
                arm.setPower(.25);
            }
            auto.movement.stopMotors();
            auto.stateTime.reset();
            while (auto.stateTime.milliseconds() < 400) {
                if (!grabbed) {
                    wobbler.setPosition(.85);
                } else {
                    wobbler.setPosition(0);
                }
            }
            if (!park) {
                while (arm.getCurrentPosition() > 110) {
                    arm.setPower(-.45);
                }
            }
            arm.setPower(0);
            break;
        }
    }

    public void grabWobble(double targetpos) {
        wobbler.setPosition(targetpos);
    }

    public void setAngle(double targetpos) {
        angler.setPosition(targetpos);
    }

    public void lifter(double targetpos) {
        lifter.setPosition(targetpos);
    }

    public void flicker(double targetpos) {
        flicker.setPosition(targetpos);
    }

    public void startShoot(double speed) {
        shoot1.setPower(-speed);
        shoot2.setPower(-speed);
    }

    public void stopShoot() {
        shoot1.setPower(0);
        shoot2.setPower(0);
    }

    public double getNeededPower(int voltageStage) {
        if(voltageStage == 4)
            return .87;
        if(voltageStage == 3)
            return .9;
        if(voltageStage == 2)
            return .94;
        if(voltageStage == 1)
            return 1;

        return .92;
    }

  /*  public void GrabBrick(double targetpos) {
        auto.runtime.reset();
        while (auto.runtime.milliseconds() < 400 && auto.opModeIsActive()) {
            clamp.setPosition(targetpos);
        }
    } */
}
