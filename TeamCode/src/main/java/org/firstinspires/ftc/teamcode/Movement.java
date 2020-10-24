package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Array;

public class Movement {
    private AutoMaster auto = null;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor lift;
    DcMotor lift2;

    public BNO055IMU imu;
    private Orientation angles;
    Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private ElapsedTime runtime = new ElapsedTime();

    public void Init(AutoMaster autoMaster, boolean foundation) {
        auto = autoMaster;
        leftBack = auto.hardwareMap.dcMotor.get("leftback");
        leftFront = auto.hardwareMap.dcMotor.get("leftfront");
        rightBack = auto.hardwareMap.dcMotor.get("rightback");
        rightFront = auto.hardwareMap.dcMotor.get("rightfront");
        lift = auto.hardwareMap.dcMotor.get("lift");
        lift2 = auto.hardwareMap.dcMotor.get("lift2");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        if (foundation)
            initLift(1, 1000, 4);

        resetLiftEncoder();
        auto.telemetry.addData("liftavg:", getEncoderAvg());
        auto.telemetry.update();
        // init motors
    }

    public void Update(double strafe, double speed) {
        // Go forward speed
        // strafe speed
        MoveInch(speed, .5, 1);
    }

    public void MoveInch(double speed, double inches, double timeoutS) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (560 / (2.95275590551 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (2.95275590551 * Math.PI)) / 3;
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP);
                leftFront.setPower(-ChangeP);
                rightFront.setPower(ChangeP);
                rightBack.setPower(ChangeP);
                auto.telemetry.addData("MotorPow:", ChangeP);
                //  if (ChangeP < .0) {

                //}
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();

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
            double kP = p / 90;
            double kD = d / 90;
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
                if (changePID < 0) {
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
            break;
        }
    }

    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public void startMotors(double left, double right) {
        while (!auto.isStopRequested() && auto.opModeIsActive()) {
            leftFront.setPower(-left);
            leftBack.setPower(-left);
            rightFront.setPower(right);
            rightBack.setPower(right);
            break;
        }
    }

    public void stopMotors() {
        while (!auto.isStopRequested() && auto.opModeIsActive()) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            break;
        }
    }

    public void Strafe(double speed, double inches) { // to go left, set speed to a negative
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            if (inches > 0) {
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                if (Math.abs(leftBack.getCurrentPosition()) >= ticks) {
                    break;
                }
            }

        }
        stopMotors();
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

    public int getLiftEncoder() {
        return lift2.getCurrentPosition();
    }

    public void resetLiftEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getEncoderAvg() {
        return (lift.getCurrentPosition() + lift2.getCurrentPosition()) / 2;
    }

    public void initLift(double speed, double encoderChange, double timeoutS) {
        runtime.reset();
        resetLiftEncoder();
        if (encoderChange < 0) {
            while (getEncoderAvg() >= encoderChange + 10 && !auto.isStopRequested()) {
                lift.setPower(-speed);
                lift2.setPower(-speed);
                auto.telemetry.addData("lift1:", lift.getCurrentPosition());
                auto.telemetry.addData("lift2:", lift2.getCurrentPosition());
                auto.telemetry.addData("encoderVal:", getEncoderAvg());
                auto.telemetry.update();
                if (getEncoderAvg() <= encoderChange + 10 || runtime.seconds() > timeoutS) {
                    break;
                }
            }
            lift2.setPower(0);
            lift.setPower(0);
        } else if (encoderChange > 0) {
            while (getEncoderAvg() <= encoderChange - 10 && !auto.isStopRequested()) {
                lift.setPower(speed);
                lift2.setPower(speed);
                auto.telemetry.addData("lift1:", lift.getCurrentPosition());
                auto.telemetry.addData("lift2:", lift2.getCurrentPosition());
                auto.telemetry.addData("encoderVal:", getEncoderAvg());
                auto.telemetry.update();
                if (getEncoderAvg() >= encoderChange - 10 || runtime.seconds() > timeoutS) {
                    break;
                }
            }
            lift.setPower(0);
            lift2.setPower(0);
        }

    }

    public void liftEncoder(double speed, double encoderChange, double timeoutS) {
        runtime.reset();
        resetLiftEncoder();
        if (encoderChange < 0) {
            while (getEncoderAvg() >= encoderChange + 10 && auto.opModeIsActive() && !auto.isStopRequested()) {
                lift.setPower(-speed);
                lift2.setPower(-speed);
                auto.telemetry.addData("lift1:", -lift.getCurrentPosition());
                auto.telemetry.addData("lift2:", lift2.getCurrentPosition());
                auto.telemetry.addData("encoderVal:", getEncoderAvg());
                auto.telemetry.update();
                if (getEncoderAvg() <= encoderChange + 10 || runtime.seconds() > timeoutS) {
                    break;
                }
            }
            lift2.setPower(0);
            lift.setPower(0);
        } else if (encoderChange > 0) {
            while (getEncoderAvg() <= encoderChange - 10 && auto.opModeIsActive() && !auto.isStopRequested()) {
                lift.setPower(speed);
                lift2.setPower(speed);
                auto.telemetry.addData("lift1:", -lift.getCurrentPosition());
                auto.telemetry.addData("lift2:", lift2.getCurrentPosition());
                auto.telemetry.addData("encoderVal:", getEncoderAvg());
                auto.telemetry.update();
                if (getEncoderAvg() >= encoderChange - 10 || runtime.seconds() > timeoutS) {
                    break;
                }
            }
            lift.setPower(0);
            lift2.setPower(0);
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

    public void GyroMoveInch(double speed, double inches, double timeoutS) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (560 / (2.95275590551 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (2.95275590551 * Math.PI)) / 3;
            double initialHeading = getGyroYaw();
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(initialHeading, getGyroYaw());
                double GyroScalePower = AngleDiff * .04;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP + GyroScalePower);
                leftFront.setPower(-ChangeP + GyroScalePower);
                rightFront.setPower(ChangeP + GyroScalePower);
                rightBack.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.addData("initHeading:", initialHeading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();

    }

    public void gyroInchHeading(double speed, double inches, double timeoutS, int heading) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (560 / (2.95275590551 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (2.95275590551 * Math.PI)) / 3;
            heading = -heading;
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = AngleDiff * .04;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP + GyroScalePower);
                leftFront.setPower(-ChangeP + GyroScalePower);
                rightFront.setPower(ChangeP + GyroScalePower);
                rightBack.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.addData("heading:", heading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();

    }

    public void rightGyroStrafe(double speed, double inches, double timeoutS, double heading) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .02;
            if (angleDiff > 3) {
                leftBack.setPower(speed + GyroScalePower);
                rightBack.setPower(speed + GyroScalePower);
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
            } else if (angleDiff < -3) {
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed + GyroScalePower);
                rightFront.setPower(-speed + GyroScalePower);
            } else {
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
            }
            if (Math.abs(leftBack.getCurrentPosition()) >= ticks || runtime.seconds() > timeoutS) {
                break;
            }
            auto.telemetry.addData("YawAngle:", getGyroYaw());
            auto.telemetry.update();

        }
        stopMotors();
    }

    public void leftGyroStrafe(double speed, double inches, double timeoutS, double heading) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .03;
            if (angleDiff > 3) {
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                leftFront.setPower(speed + GyroScalePower);
                rightFront.setPower(speed + GyroScalePower);
            } else if (angleDiff < -3) {
                leftBack.setPower(-speed + GyroScalePower);
                rightBack.setPower(-speed + GyroScalePower);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
            } else {
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
            }
            if (Math.abs(leftBack.getCurrentPosition()) >= ticks || runtime.seconds() > timeoutS) {
                break;
            }

            auto.telemetry.addData("YawAngle:", getGyroYaw());
            auto.telemetry.update();

        }
        stopMotors();
    }

    public void foundationForward(double speed, double timeoutInch, double timeoutS, int heading) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = timeoutInch * (560 / (2.95275590551 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (2.95275590551 * Math.PI)) / 3;
            heading = -heading;
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = AngleDiff * .04;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP + GyroScalePower);
                leftFront.setPower(-ChangeP + GyroScalePower);
                rightFront.setPower(ChangeP + GyroScalePower);
                rightBack.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.addData("heading:", heading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if ((!auto.mechanisms.touch1.isPressed() && !auto.mechanisms.touch2.isPressed()) || Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();


    }

    public void MoveInchV2(double speed, double inches, double timeoutS) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (560 / (3.85826771654 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (3.85826771654 * Math.PI)) / 3;
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP);
                leftFront.setPower(-ChangeP);
                rightFront.setPower(ChangeP);
                rightBack.setPower(ChangeP);
                auto.telemetry.addData("MotorPow:", ChangeP);
                //  if (ChangeP < .0) {

                //}
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();

    }

    public void GyroMoveInchV2(double speed, double inches, double timeoutS) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (560 / (3.85826771654 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (3.85826771654 * Math.PI)) / 3;
            double initialHeading = getGyroYaw();
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(initialHeading, getGyroYaw());
                double GyroScalePower = AngleDiff * .04;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP + GyroScalePower);
                leftFront.setPower(-ChangeP + GyroScalePower);
                rightFront.setPower(ChangeP + GyroScalePower);
                rightBack.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.addData("initHeading:", initialHeading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();

    }

    public void gyroInchHeadingV2(double speed, double inches, double timeoutS, int heading) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = inches * (560 / (3.85826771654 * Math.PI));
            double kP = speed / 10;
            double stopVal = (560 / (3.85826771654 * Math.PI)) / 3;
            heading = -heading;
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (2.95275590551));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = AngleDiff * .04;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP + GyroScalePower);
                leftFront.setPower(-ChangeP + GyroScalePower);
                rightFront.setPower(ChangeP + GyroScalePower);
                rightBack.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.addData("heading:", heading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if (Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();

    }

    public void leftGyroStrafeV2(double speed, double inches, double timeoutS, double heading) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (3.85826771654 * Math.PI));
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .05;
            if (angleDiff > 2) {
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                leftFront.setPower(speed + GyroScalePower);
                rightFront.setPower(speed + GyroScalePower);
            } else if (angleDiff < -2) {
                leftBack.setPower(-speed + GyroScalePower);
                rightBack.setPower(-speed + GyroScalePower);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
            } else {
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
            }
            if (Math.abs(leftBack.getCurrentPosition()) >= ticks || runtime.seconds() > timeoutS) {
                break;
            }

            auto.telemetry.addData("YawAngle:", getGyroYaw());
            auto.telemetry.update();

        }
        stopMotors();
    }

    public void rightGyroStrafeV2(double speed, double inches, double timeoutS, double heading) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (3.85826771654 * Math.PI));
        heading = -heading;
        //runtime isn't used, this is just a backup call which we don't need

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        while (Math.abs(leftBack.getCurrentPosition()) < ticks && auto.opModeIsActive()) {
            double angleDiff = GimbleCalc(heading, getGyroYaw());
            double GyroScalePower = angleDiff * .03;
            if (angleDiff > 2) {
                leftBack.setPower(speed + GyroScalePower);
                rightBack.setPower(speed + GyroScalePower);
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
            } else if (angleDiff < -2) {
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed + GyroScalePower);
                rightFront.setPower(-speed + GyroScalePower);
            } else {
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
            }
            if (Math.abs(leftBack.getCurrentPosition()) >= ticks || runtime.seconds() > timeoutS) {
                break;
            }
            auto.telemetry.addData("YawAngle:", getGyroYaw());
            auto.telemetry.update();

        }
        stopMotors();
    }

    public void foundationForwardV2(double speed, double timeoutInch, double timeoutS, int heading) {
        while (auto.opModeIsActive() && !auto.isStopRequested()) {
            // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
            double ticks = timeoutInch * (560 / (3.85826771654 * Math.PI));
            double kP = speed / 20;
            double stopVal = (560 / (3.85826771654 * Math.PI)) / 3;
            heading = -heading;
            //runtime isn't used, this is just a backup call which we don't need
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
            //if the position is less than the number of inches, than it sets the motors to speed
            while (Math.abs(leftBack.getCurrentPosition()) <= ticks - stopVal && auto.opModeIsActive()) {
                double error = (ticks - Math.abs(rightBack.getCurrentPosition())) / (560 / (3.85826771654));
                double ChangeP = error * kP;
                double AngleDiff = GimbleCalc(heading, getGyroYaw());
                double GyroScalePower = AngleDiff * .02;
                if (ChangeP > 1)
                    ChangeP = ChangeP / ChangeP;
                leftBack.setPower(-ChangeP + GyroScalePower);
                leftFront.setPower(-ChangeP + GyroScalePower);
                rightFront.setPower(ChangeP + GyroScalePower);
                rightBack.setPower(ChangeP + GyroScalePower);
                auto.telemetry.addData("MotorPow:", ChangeP);
                auto.telemetry.addData("heading:", heading);
                auto.telemetry.addData("YawAngle:", getGyroYaw());
                auto.telemetry.update();
                if ((auto.mechanisms.touch1.isPressed() && auto.mechanisms.touch2.isPressed()) || Math.abs(ChangeP) < .15 || runtime.seconds() >= timeoutS) {
                    break;
                }
            }
            break;
        }
        stopMotors();
    }

}
