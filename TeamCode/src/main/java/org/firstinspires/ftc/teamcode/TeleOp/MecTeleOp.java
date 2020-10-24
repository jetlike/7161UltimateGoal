package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecanumTeleOp", group = "godly")
public class MecTeleOp extends OpMode {

    DcMotor lift;           //declaring stuff
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
   // DcMotor grablift;

    Servo clamp;
    double clampb = 0;
    double clampa = 0;
    boolean clampup = false;

    Servo foundation1;
    Servo foundation2;
    double foundb = 0;
    boolean foundup = true;

    Servo capstone;
    double capb = 0;
    boolean capup = true;


    private ElapsedTime runtime = new ElapsedTime();


    public void init() {
        lift = hardwareMap.dcMotor.get("lift");        //initialize motors, servos
        lf = hardwareMap.dcMotor.get("leftfront");
        rf = hardwareMap.dcMotor.get("rightfront");
        lb = hardwareMap.dcMotor.get("leftback");
        rb = hardwareMap.dcMotor.get("rightback");
    //    grablift = hardwareMap.dcMotor.get("grablift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  grablift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //grablift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        clamp = hardwareMap.servo.get("clamp");
        foundation1 = hardwareMap.servo.get("found1");
        foundation2 = hardwareMap.servo.get("found2");
        capstone = hardwareMap.servo.get("capstone");


        foundation1.setPosition(1);  //when foundup is true
        foundation2.setPosition(0);
        capstone.setPosition(0.3); // when capup true
        telemetry.addData("gamepad1leftstickx:", gamepad1.left_stick_x);

    }

    public void loop() {

        telemetry.addData("gamepad1leftstickx:", gamepad1.left_stick_x);
        telemetry.addData("gamepad1leftsticky:", gamepad1.left_stick_y);
        telemetry.addData("gamepad1rightstickx:", gamepad1.right_stick_x);


        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
            double FLP = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double FRP = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double BLP = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double BRP = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;

            double max = Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.max(Math.abs(BLP), Math.abs(BRP)));

            if (max > 1) {
                FLP /= max;
                FRP /= max;
                BLP /= max;
                BRP /= max;
            }

            if (gamepad1.right_trigger > 0.1) {
                lf.setPower(FLP * 0.35);
                rf.setPower(FRP * 0.35);
                lb.setPower(BLP * 0.35);
                rb.setPower(BRP * 0.35);
                telemetry.addData("FrontLeftPow:", FLP * 0.35);
                telemetry.addData("FrontRightPow:", FRP * 0.35);
                telemetry.addData("BackLeftPow:", BLP * 0.35);
                telemetry.addData("BackRightPow:", BRP * 0.35);
            } else {
                lf.setPower(FLP);
                rf.setPower(FRP);
                lb.setPower(BLP);
                rb.setPower(BRP);

            }

        } else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }

        if (Math.abs(gamepad2.left_stick_y) > .1) {            //lift code;easy stuff
            lift.setPower(-gamepad2.left_stick_y);
           // grablift.setPower(-gamepad2.left_stick_y);
            telemetry.addData("Lift Value:", -gamepad2.left_stick_y); //add telemetry to see how much power lift is getting
            telemetry.addData("EncoderVal:", lift.getCurrentPosition()); //encoder check 
        } else if (Math.abs(gamepad2.right_stick_y) > .1) {            //lift code;easy stuff
          //  grablift.setPower(-gamepad2.right_stick_y);
            telemetry.addData("Lift Value:", gamepad2.left_stick_y); //add telemetry to see how much power lift is getting
            telemetry.addData("EncoderVal:", lift.getCurrentPosition()); //encoder check
        } else {
            lift.setPower(0);
          //  grablift.setPower(0);
            telemetry.addData("Lift Value:", gamepad2.right_stick_y); //add telemetry to see how much power lift is getting
            telemetry.addData("EncoderVal:", lift.getCurrentPosition());

        }

        if (gamepad2.a) {                                              //clamp code, checks if the a button has been pressed
            if (clampup && runtime.milliseconds() > clampb + 500) {      //once pressed, will check whether clampb is true or false
                clamp.setPosition(.6);                       //makes movements based on the clampb boolean
                clampup = false;                              // checks if the last time you've hit the button has been more than
                runtime.reset();                            // x amount of seconds, so it doesn't jitter
                telemetry.addData("ClampPos:", 0.6); //add telemetry to see where clamp is positioned
            } else if (!clampup && runtime.milliseconds() > clampb + 500) {
                clamp.setPosition(0.35);
                clampup = true;
                runtime.reset();
                telemetry.addData("ClampPos:", 0.1); //add telemetry to see where clamp is positioned
            }
        }

        if (gamepad2.b) {                                         //foundation servo code, checks if the b butto n has been pressed
            if (foundup && runtime.milliseconds() > foundb + 500) {
                foundation1.setPosition(0.5);             //once pressed, will check whether or not foundup is true or false
                foundation2.setPosition(0.7);              //makes movements based on the foundup boolean
                foundup = false;                        // checks if the last time you've hit the button has been more than
                runtime.reset();                        // x amount of seconds, so it doesn't jitter
                telemetry.addData("FoundationPos:", 0.8); //add telemetry to ses where foundation servos are positioned
            } else if (!foundup && runtime.milliseconds() > foundb + 500) {
                foundation1.setPosition(1);
                foundation2.setPosition(0);
                foundup = true;
                runtime.reset();
                telemetry.addData("FoundationPos:", 0.1); //add telemetry to ses where foundation servos are positioned
            }
        }

        if (gamepad2.x) {
            if (capup && runtime.milliseconds() > capb + 500) {
                capstone.setPosition(0.8);
                capup = false;
                runtime.reset();
                telemetry.addData("CapstonePos:", 1);
            } else if (!capup && runtime.milliseconds() > capb + 500) {
                capstone.setPosition(0.3);
                capup = true;
                runtime.reset();
                telemetry.addData("CapstonePos:", 0);
            }
        }

        if (gamepad2.y) {
            if (runtime.milliseconds() > clampa + 500) {
                clamp.setPosition(0);
                runtime.reset();
            }
        }


        telemetry.addData("liftencoderVal:", lift.getCurrentPosition());
        telemetry.addData("servopos:", clamp.getPosition());

    }

    public void stop() {


    }
}

