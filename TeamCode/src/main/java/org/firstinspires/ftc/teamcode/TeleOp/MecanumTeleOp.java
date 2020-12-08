package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    DcMotor shoot1;
    DcMotor shoot2;
    DcMotor intake;
    Servo lifter;
    Servo flicker;
    Servo wobbler;


    boolean shoot = false;
    boolean lift = false;
    boolean flick = false;
    boolean wobble = true;

    public void init() {
        fl = hardwareMap.dcMotor.get("FL");   //hardware map
        fr = hardwareMap.dcMotor.get("FR");
        bl = hardwareMap.dcMotor.get("BL");
        br = hardwareMap.dcMotor.get("BR");
        shoot1 = hardwareMap.dcMotor.get("s1");
        shoot2 = hardwareMap.dcMotor.get("s2");
        intake = hardwareMap.dcMotor.get("intake");
        lifter = hardwareMap.servo.get("lifter");
        flicker = hardwareMap.servo.get("flicker");
        wobbler = hardwareMap.servo.get("wobbler");


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //define motor settings
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
        lifter.setPosition(1);
        flicker.setPosition(.7);
        wobbler.setPosition(.6);


    }

    public void loop() {
        if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {      //check for inputs
            double FLP = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;          //calculate power for each motor
            double FRP = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double BLP = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double BRP = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;

            double max = Math.max(Math.max(BLP, BRP), Math.max(FLP, FRP)); //find max value

            if (max > 1) {    //if max value over 1, divide everything by max to get correct range (0-1 power)
                FLP /= max;
                FRP /= max;
                BLP /= max;
                BRP /= max;
            }

            if (gamepad1.right_trigger > .1) {    //slow mode for accurate movements
                fl.setPower(FLP * .35);
                fr.setPower(FRP * .35);
                bl.setPower(BLP * .35);
                br.setPower(BRP * .35);
                telemetry.addData("FLP:", FLP * .35);
                telemetry.addData("FRP:", FRP * .35);
                telemetry.addData("BLP:", BLP * .35);
                telemetry.addData("BRP:", BRP * .35);
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

        if (gamepad2.b && !shoot) {
            shoot1.setPower(1);
            shoot2.setPower(1);
            shoot = true;
            telemetry.addData("ShooterOn:", true);
        } else if (gamepad2.b && shoot) {
            shoot1.setPower(0);
            shoot2.setPower(0);
            shoot = false;
            telemetry.addData("ShooterOn:", false);
        }

        if (gamepad1.right_bumper) {
            intake.setPower(1);
            telemetry.addData("IntakePow:", 1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(0);
            telemetry.addData("IntakePow:", 0);
        } else if (gamepad1.right_bumper && gamepad1.left_bumper) {
            intake.setPower(-1);
            telemetry.addData("IntakePow:", -1);
        }

        if (gamepad2.a && !lift) {
            lifter.setPosition(.7);
            lift = true;
            telemetry.addData("LiftPosition:", "Lifted/0.7");
        } else if (gamepad2.a && lift) {
            lifter.setPosition(1);
            lift = false;
            telemetry.addData("LiftPosition:", "Not Lifted/1");

        }

        if (gamepad2.b && !flick) {
            lifter.setPosition(.5);
            flick = true;
            telemetry.addData("FlickPosition:", "Flicked/0.5");
        } else if (gamepad2.b && flick) {
            lifter.setPosition(1);
            flick = false;
            telemetry.addData("FlickPosition:", "Not Flicked/1");
        }

        if (gamepad2.right_bumper && !wobble) {
            wobbler.setPosition(.6);
            wobble = true;
            telemetry.addData("WobblerPosition:", "Grabbed/0.6");
        } else if (gamepad2.right_bumper && wobble) {
            wobbler.setPosition(0);
            wobble = false;
            telemetry.addData("WobblerPosition:", "Not Grabbed/0");
        }

        telemetry.update();
    }

    public void stop() {

    }
}
