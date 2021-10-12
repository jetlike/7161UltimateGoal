package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MecanumTeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor verticalLeft;
    DcMotor verticalRight;
    DcMotor horizontal;
    DcMotorEx shoot1;
    DcMotorEx shoot2;
    DcMotor intake;
    DcMotor arm;
    Servo lifter;
    Servo flicker;
    Servo wobbler;
    Servo angler;

    double motorVelocity = -2500;


    private ElapsedTime runtimeflick = new ElapsedTime();
    private ElapsedTime runtimelift = new ElapsedTime();
    private ElapsedTime runtimegrab = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();


    boolean shoot = false;
    boolean lift = false;
    boolean flick = false;
    boolean wobble = true;

    private void  flicktime() {
        runtime.reset();
        for (; runtime.milliseconds() < 500;) {
            flicker.setPosition(.66);
        }
        runtime.reset();
        for (; runtime.milliseconds() < 1000;) {
            flicker.setPosition(1);
        }
    }

    private void shoot() {
        for (int i = 0; i<3; i++) {
                flicktime();
        }
    }

    public void init() {
        fl = hardwareMap.dcMotor.get("FL");   //hardware a map
        fr = hardwareMap.dcMotor.get("FR");
        bl = hardwareMap.dcMotor.get("BL");
        br = hardwareMap.dcMotor.get("BR");
        verticalLeft = hardwareMap.dcMotor.get("FL");
        verticalRight = hardwareMap.dcMotor.get("FR");
        horizontal = hardwareMap.dcMotor.get("BR");
        shoot1 = hardwareMap.get(DcMotorEx.class,"s1");
        shoot2 = hardwareMap.get(DcMotorEx.class,"s2");
        intake = hardwareMap.dcMotor.get("intake");
        lifter = hardwareMap.servo.get("lifter");
        flicker = hardwareMap.servo.get("flicker");
        wobbler = hardwareMap.servo.get("wobbler");
        angler = hardwareMap.servo.get("angler");
        arm = hardwareMap.dcMotor.get("arm");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //define motor settings
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2.setDirection(DcMotorSimple.Direction.FORWARD);




    }


    public void loop() {

        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {      //check for inputs
            double FLP = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;          //calculate power for each motor
            double FRP = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double BLP = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double BRP = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

            double max = Math.max(Math.max(BLP, BRP), Math.max(FLP, FRP)); //find max value

            if (max > 1) {    //if max value over 1, divide everything by max to get correct range (0-1 power)
                FLP /= max;
                FRP /= max;
                BLP /= max;
                BRP /= max;
            }

            if (gamepad1.right_trigger > .1) {    //slow mode for accurate movements
                fl.setPower(FLP * .5);
                fr.setPower(FRP * .5);
                bl.setPower(BLP * .5);
                br.setPower(BRP * .5);
                telemetry.addData("FLP:", FLP * .5);
                telemetry.addData("FRP:", FRP * .5);
                telemetry.addData("BLP:", BLP * .5);
                telemetry.addData("BRP:", BRP * .5);
            } else {
                fl.setPower(FLP);
                fr.setPower(FRP);
                bl.setPower(BLP);
                br.setPower(BRP);
                telemetry.addData("FLP:", FLP);
                telemetry.addData("FRP:", FRP);
                telemetry.addData("BLP:", BLP);
                telemetry.addData("BRP:", BRP);            }

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        if (Math.abs(gamepad2.right_trigger) > .1) {
            shoot1.setPower(-1);
            shoot2.setPower(-1);
            telemetry.addData("ShooterOn:", 1);
            telemetry.addData("Shoot1Encoder:", shoot1.getCurrentPosition());
            telemetry.addData("Shoot2Encoder:", shoot2.getCurrentPosition());
        } else {
            shoot1.setPower(0);
            shoot2.setPower(0);
        }

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            intake.setPower(-1);
            telemetry.addData("IntakePow:", -1);
        } else if (gamepad1.right_bumper && !lift) {
            intake.setPower(1);
            telemetry.addData("IntakePow:", 1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(0);
            telemetry.addData("IntakePow:", 0);
        }

        if (gamepad2.a && !lift && runtimelift.milliseconds() > 600) {
            lifter.setPosition(.6443);
            lift = true;
            runtimelift.reset();
            telemetry.addData("LiftPosition:", "Lifted/.65");
        } else if (gamepad2.a && lift && runtimelift.milliseconds() > 600) {
            lifter.setPosition(.9);
            lift = false;
            runtimelift.reset();
            telemetry.addData("LiftPosition:", "Not Lifted/.9");

        }

        if (gamepad2.b && !flick && runtimeflick.milliseconds() > 600) {
            flicker.setPosition(.66);
            flick = true;
            runtimeflick.reset();
            telemetry.addData("FlickPosition:", "Flicked/0.5");
        } else if (gamepad2.b && flick && runtimeflick.milliseconds() > 600) {
            flicker.setPosition(1);
            flick = false;
            runtimeflick.reset();
            telemetry.addData("FlickPosition:", "Not Flicked/1");
        }

        if (gamepad2.right_bumper && !wobble && runtimegrab.milliseconds() > 600) {
            wobbler.setPosition(.85);
            wobble = true;
            runtimegrab.reset();
            telemetry.addData("WobblerPosition:", "Grabbed/0.6");
        } else if (gamepad2.right_bumper && wobble && runtimegrab.milliseconds() > 600) {
            wobbler.setPosition(0);
            wobble = false;
            runtimegrab.reset();
            telemetry.addData("WobblerPosition:", "Not Grabbed/0");
        }

        if (Math.abs(gamepad2.left_stick_y) > .1) {
            arm.setPower(gamepad2.left_stick_y * -.45);
            telemetry.addData("ArmPow", gamepad2.left_stick_y * -.45);
            telemetry.addData("ArmPo:", arm.getCurrentPosition());
        } else {
            arm.setPower(0);
        }

        if (gamepad2.dpad_down) {
            angler.setPosition(0);
            telemetry.addData("Angle Change:", "down");
        } else if (gamepad2.dpad_up) {
            angler.setPosition(.75);
            telemetry.addData("Angle Change:", "up");
        } else if (gamepad2.dpad_right) {
            angler.setPosition(.74);
            telemetry.addData("Angle Change:", "powershot");
        }

        if (gamepad2.x) {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            shoot();
        }

     //   telemetry.addData("verticalLeft:", verticalLeft.getCurrentPosition());
      //  telemetry.addData("verticalRight:", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal:", horizontal.getCurrentPosition());
        telemetry.addData("verticalRight:", verticalRight.getCurrentPosition());
        telemetry.addData("verticalLeft:", verticalLeft.getCurrentPosition());
        telemetry.addData("FL:", fl.getCurrentPosition());
        telemetry.addData("FR:", fr.getCurrentPosition());
        telemetry.addData("BL:", bl.getCurrentPosition());
        telemetry.addData("BR:", br.getCurrentPosition());
        telemetry.update();
    }

    public void stop() {

    }
/*
    private static class macro extends Thread {
        public macro() {

        }
        @Override
        public void run() {
            try {
                while (!isInterrupted()) {
                        shoot();
                        break;
                    }
                }
            } catch (InterruptedException e) {
                telemetry.addData("macro:", "error");
            }
        }
    }
    */
}
