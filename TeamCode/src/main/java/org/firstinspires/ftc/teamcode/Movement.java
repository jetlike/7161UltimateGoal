package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Movement {
    private AutoMaster auto;
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor verticalLeft;
    DcMotor verticalRight;
    DcMotor horizontal;


    public BNO055IMU imu;
    private Orientation angles;
    Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private ElapsedTime runtime = new ElapsedTime();

    public void Init(AutoMaster autoMaster) {
        auto = autoMaster;
        fl = auto.hardwareMap.dcMotor.get("FL");   //hardware a map
        fr = auto.hardwareMap.dcMotor.get("FR");
        bl = auto.hardwareMap.dcMotor.get("BL");
        br = auto.hardwareMap.dcMotor.get("BR");
        verticalLeft = auto.hardwareMap.dcMotor.get("FL");
        verticalRight = auto.hardwareMap.dcMotor.get("FR");
        horizontal = auto.hardwareMap.dcMotor.get("BR");

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
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = this.auto.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!auto.isStopRequested() && !imu.isGyroCalibrated()) {
            auto.sleep(50);
            auto.idle();
        }
        auto.telemetry.update();
        // init motors
    }

    public void resetEncoders() {
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveInch(double speed, double inches, double timeoutS) {
        resetEncoders();
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            double ticks = inches * (537.6 / (3.93701 * Math.PI));
            double kP = speed / 13;
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(bl.getCurrentPosition()) <= ticks && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(br.getCurrentPosition())) / (537.6 / (3.93701));
                double ChangeP = error * kP;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                bl.setPower(-ChangeP);
                fl.setPower(ChangeP);
                fr.setPower(ChangeP);
                br.setPower(-ChangeP);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    stopMotors();
                    break;
                }
            }
            break;
        }


    }

    public void rightGyroStrafe(double speed, double inches, double timeoutS, double heading) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (1440 / (1.49606 * Math.PI));
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need
        double kP = speed / (3.3);
        resetOdomEncoders();
     //   double verticalStart = verticalOdomAverage();
     //   double verticalPower;
        runtime.reset();

        while (Math.abs(horizontal.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            double error = (ticks - Math.abs(horizontal.getCurrentPosition()))/1440;
            double ChangeP = kP * error;
            if (ChangeP > 1){
                ChangeP /= ChangeP;
            } else if (ChangeP < .23) {
                double x = .23/ChangeP;
                ChangeP *= x;
            }
            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .04;
        //    double verticalDiff = verticalOdomAverage() - verticalStart;
        /*    if (Math.abs(verticalDiff) > 340) {
                verticalPower = verticalDiff * .02;
            } else {
                verticalPower = 0;
            }
         */
            if(angleDiff > 2) {
                bl.setPower(ChangeP);
                br.setPower(ChangeP);
                fl.setPower(-ChangeP + GyroScalePower);
                fr.setPower(-ChangeP + GyroScalePower);
            } else if (angleDiff < -2) {
                bl.setPower(ChangeP + GyroScalePower);
                br.setPower(ChangeP + GyroScalePower);
                fl.setPower(-ChangeP);
                fr.setPower(-ChangeP);
            } else {
                bl.setPower(ChangeP);
                br.setPower(ChangeP);
                fl.setPower(-ChangeP);
                fr.setPower(-ChangeP);
            }
            if (Math.abs(horizontal.getCurrentPosition()) >= ticks || runtime.seconds() > timeoutS) {
                stopMotors();
                break;
            }

            auto.telemetry.addData("horizontal:", horizontal.getCurrentPosition());
            auto.telemetry.addData("YawAngle:", getGyroYaw());
            auto.telemetry.update();

        }
        stopMotors();

    }

    public void leftGyroStrafe(double speed, double inches, double timeoutS, double heading) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (1440 / (1.49606 * Math.PI));
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need
        double kP = speed / 3.3;
        resetOdomEncoders();
        runtime.reset();
        while (Math.abs(horizontal.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            double error = (ticks - Math.abs(horizontal.getCurrentPosition()))/1440;
            double ChangeP = kP * error;
            if (ChangeP > 1){
                ChangeP /= ChangeP;
            } else if (ChangeP < .23) {
                double x = .23/ChangeP;
                ChangeP *= x;
            }
            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .04;

            if (ChangeP > 1){
                ChangeP /= ChangeP;
            }
            if (angleDiff > 2) {
                bl.setPower(-ChangeP + GyroScalePower);
                br.setPower(-ChangeP + GyroScalePower);
                fl.setPower(ChangeP);
                fr.setPower(ChangeP);
            } else if (angleDiff < -2) {
                bl.setPower(-ChangeP);
                br.setPower(-ChangeP);
                fl.setPower(ChangeP + GyroScalePower);
                fr.setPower(ChangeP + GyroScalePower);
            } else {
                bl.setPower(-ChangeP);
                br.setPower(-ChangeP);
                fl.setPower(ChangeP);
                fr.setPower(ChangeP);
            }
            if (Math.abs(horizontal.getCurrentPosition()) >= ticks || runtime.seconds() > timeoutS) {
                stopMotors();
                break;
            }
            auto.telemetry.addData("horizontal:", horizontal.getCurrentPosition());
            auto.telemetry.addData("YawAngle:", getGyroYaw());
            auto.telemetry.addData("finished:", true);
            auto.telemetry.update();

        }
        stopMotors();

    }

    public void gyroInch(double speed, double inches, double timeoutS, int heading) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (537.6 / (3.93701 * Math.PI));
            double kP = speed / 15;
            heading = -heading;
            //runtime isn't used, this is just a backup call which we don't need
            resetEncoders();
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(bl.getCurrentPosition()) <= ticks && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(br.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = AngleDiff * .02;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                bl.setPower(-ChangeP + GyroScalePower);
                fl.setPower(-ChangeP + GyroScalePower);
                fr.setPower(ChangeP + GyroScalePower);
                br.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPowLeft:", -ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPowRight:", ChangeP + GyroScalePower);
                auto.telemetry.addData("heading:", heading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    stopMotors();
                    break;
                }
            }
            break;
        }
        stopMotors();


    }

    public void resetOdomEncoders() {
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int verticalOdomAverage() {
        return (Math.abs(verticalRight.getCurrentPosition()) + Math.abs(verticalLeft.getCurrentPosition())) / 2;
    }

    public void gyroEncoderInch(double speed, double inches, double timeoutS, int heading) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (1440 / (1.49606 * Math.PI));
            double kP = speed / 14;
            heading = -heading;
            //runtime isn't used, this is just a backup call which we don't need
            //if the position is less than the number of inches, than it sets the motors to speed
            runtime.reset();
            resetOdomEncoders();
            while (Math.abs(verticalOdomAverage()) <= ticks && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(verticalOdomAverage()))/1440;
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = AngleDiff * .04;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                bl.setPower(-ChangeP + GyroScalePower);
                fl.setPower(-ChangeP + GyroScalePower);
                fr.setPower(ChangeP + GyroScalePower);
                br.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPowLeft:", -ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPowRight:", ChangeP + GyroScalePower);
                auto.telemetry.addData("heading:", heading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.addData("encoders:", verticalOdomAverage());
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    stopMotors();
                    break;
                }
            }
            stopMotors();
            break;
        }



    }


    public double getTrueDiff(double origAngle) {
        double currAngle = getGyroYaw();
        if (currAngle >= 0 && origAngle >= 0 || currAngle <= 0 && origAngle <= 0)
            return (currAngle - origAngle);
        else if (Math.abs(currAngle - origAngle) <= 180)
            return (currAngle - origAngle);
        else if (currAngle > origAngle)
            return -(360 - (currAngle - origAngle));
        else
            return (360 + (currAngle - origAngle));
    }

    public void turnPD(double angle, double p, double d, double timeout) {//.4p and .45d for 90
        while (auto.opModeIsActive() && !auto.isStopRequested()) {        // big turns = big/avg val
            runtime.reset();                                              // small turns = small val
            double kP = p / 33;
            double kD = d / .70;
            double currentTime = runtime.milliseconds();
            double pastTime = 0;
            double prevAngleDiff = getTrueDiff(-angle);
            double angleDiff = prevAngleDiff;
            double changePID = 0;
            while (Math.abs(angleDiff) > .95 && runtime.seconds() < timeout && auto.opModeIsActive()) {
                pastTime = currentTime;
                currentTime = runtime.milliseconds();
                double dT = currentTime - pastTime;
                angleDiff = getTrueDiff(-angle);
                changePID = (angleDiff * kP) + ((angleDiff - prevAngleDiff) / dT * kD);
                if (changePID <= 0) {
                    startMotors(changePID - .10, -changePID + .10);
                } else {
                    startMotors(changePID + .10, -changePID - .10);
                }
                   auto.telemetry.addData("P", (angleDiff * kP));
                   auto.telemetry.addData("D", ((Math.abs(angleDiff) - Math.abs(prevAngleDiff)) / dT * kD));
                   auto.telemetry.addData("angle:", getGyroYaw());
                   auto.telemetry.update();

                prevAngleDiff = angleDiff;
            }
            stopMotors();

           angleDiff = getTrueDiff(-angle);
            if (!(Math.abs(angleDiff) > .95)) {
                break;
            }


        }
    }

    public void odomTune() {
        resetOdomEncoders();
        auto.runtime.reset();
        while (auto.runtime.milliseconds() < 2000) {
            fl.setPower(-.3);
            fr.setPower(.3);   //3
            bl.setPower(-.3);
            br.setPower(.3);
            auto.telemetry.addData("horizontal:", horizontal.getCurrentPosition());
            auto.telemetry.addData("verticalR:", verticalRight.getCurrentPosition());
            auto.telemetry.addData("verticalL:", verticalLeft.getCurrentPosition());
            auto.telemetry.update();
        }
        stopMotors();
        while (!auto.isStopRequested()){
            auto.telemetry.addData("horizontal:", horizontal.getCurrentPosition());
            auto.telemetry.addData("verticalR:", verticalRight.getCurrentPosition());
            auto.telemetry.addData("verticalL:", verticalLeft.getCurrentPosition());
            auto.telemetry.update();
        }

    }

    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public double getGyroRoll() {
        updateGyroValues();
        return angles.secondAngle;
    }

    public double getGyroPitch() {
        updateGyroValues();
        return angles.thirdAngle;
    }

    public void startMotors(double left, double right) {                      //TODO: test motors and if this is the correct way to move
        if (!auto.isStopRequested() && auto.opModeIsActive()) {
            bl.setPower(-left);
            fl.setPower(-left);
            fr.setPower(right);
            br.setPower(right);
        }
    }

    public void stopMotors() {
        if (!auto.isStopRequested() && auto.opModeIsActive()) {
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        }
    }


    public void reversearcturnPD(double angle, double p, double d, double timeout) {//.4p and .45d for 90
        while (auto.opModeIsActive() && !auto.isStopRequested()) {        // big turns = big/avg val
            runtime.reset();                                              // small turns = small val
            double kP = p / 90;
            double kD = d / 90;
            double currentTime = runtime.milliseconds();
            double pastTime = 0;
            double prevAngleDiff = getTrueDiff(-angle);
            double angleDiff = prevAngleDiff;
            double changePID = 0;
            while (Math.abs(angleDiff) > .5 && runtime.seconds() < timeout && auto.opModeIsActive()) {
                pastTime = currentTime;
                currentTime = runtime.milliseconds();
                double dT = currentTime - pastTime;
                angleDiff = getTrueDiff(-angle);
                changePID = (angleDiff * kP) + ((angleDiff - prevAngleDiff) / dT * kD);
                if (changePID < 0) {
                    startMotors(changePID + .10, 0);
                } else {
                    startMotors(0, -changePID - .10);
                }
                /*
                auto.telemetry.addData("P", (angleDiff * kP));
                auto.telemetry.addData("D", ((Math.abs(angleDiff) - Math.abs(prevAngleDiff)) / dT * kD));
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                 */
                prevAngleDiff = angleDiff;
            }
            stopMotors();
            break;
        }
    }

    public void arcturnPD(double angle, double p, double d, double timeout) {//.4p and .45d for 90
        while (auto.opModeIsActive() && !auto.isStopRequested()) {        // big turns = big/avg val
            runtime.reset();                                              // small turns = small val
            double kP = p / 90;
            double kD = d / 90;
            double currentTime = runtime.milliseconds();
            double pastTime = 0;
            double prevAngleDiff = getTrueDiff(-angle);
            double angleDiff = prevAngleDiff;
            double changePID = 0;
            while (Math.abs(angleDiff) > .5 && runtime.seconds() < timeout && auto.opModeIsActive()) {
                pastTime = currentTime;
                currentTime = runtime.milliseconds();
                double dT = currentTime - pastTime;
                angleDiff = getTrueDiff(-angle);
                changePID = (angleDiff * kP) + ((angleDiff - prevAngleDiff) / dT * kD);
                if (changePID < 0) {
                    startMotors(changePID + .10, (-changePID - .10) * .8);
                } else {
                    startMotors((changePID - .10) * .8, -changePID + .10);
                }
                //   auto.telemetry.addData("P", (angleDiff * kP));
                //   auto.telemetry.addData("D", ((Math.abs(angleDiff) - Math.abs(prevAngleDiff)) / dT * kD));

                prevAngleDiff = angleDiff;
            }
            stopMotors();
            break;
        }
    }








    public double GimbleCalc(double initVal, double curVal) {
        double angleDiff;
        if (initVal - curVal >= 180)
            angleDiff = -1 * ((360 + curVal) - initVal);
        else if (curVal - initVal >= 180)
            angleDiff = (360 + initVal) - curVal;
        else
            angleDiff = initVal - curVal;

        return angleDiff;
    }





}
