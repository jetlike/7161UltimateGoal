/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "AutoMaster", group = "godlyAuto")
@Disabled
public abstract class AutoMaster extends LinearOpMode {


    private WhatToDo whatToDo = new WhatToDo();
    public Vision vision = new Vision();
    private Movement movement = new Movement();
    public Mechanisms mechanisms = new Mechanisms();
    public Bitmap bitmap = new Bitmap();


    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime stateTime = new ElapsedTime();

    String step = "forward1";

    double brickDown = .6;
    double brickUp = .35;
    double foundation1down = .5; //found1 is on the right so starts at 0, goes to 1 to grab
    double foundation2down = .6;
    double foundation1up = 0;
    double foundation2up = 1;

    double turn90p = .53;
    double turn90d = .93;

    double turn105p = .46;
    double turn105d = .9;


    public void QuarryAuto(boolean red) throws InterruptedException {
        while (!isStopRequested()) {
            movement.Init(this, true);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        boolean firstbrick = true;
        double strafe = 0;
        double deliver1stbrick = 0;
        double get2ndbrick = 0;
        double deliver2ndbrick = 0;
        boolean pos1and4 = false;
        boolean pos3and6 = false;
        double distance = 0;
        double strafeSpeed = 0.6;
        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData("servopos:", mechanisms.clamp.getPosition());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (step.equals("test")) {
                movement.gyroInchHeading(-.2, 50, 10, -90);
                sleep(1000);
                movement.rightGyroStrafe(.2, 20, 10, -90);
                sleep(1000);
                movement.turnPD(90, .68, .9, 5);
                break;
            }
            if (step.equals("debug")) {
                vision.Update();
                whatToDo.Update(vision.results.left, vision.results.right, vision.results.top, vision.results.bottom, vision.results.confidence, red, firstbrick);
            }

            if (step.equals("forward1")) {
                if (!red) {
                    movement.MoveInch(1, 16, 3);
                } else {
                    movement.leftGyroStrafe(.4, 8, 5, 3);
                    sleep(200);
                    movement.GyroMoveInch(1, 18, 3);
                }
                Switchstate("bitmap");
            }
           /* if (step.equals("lift")) {
                movement.liftEncoder(1, 500, 2);
                Switchstate("bitmap");
            }*/
/*
            if (step.equals("clampdown")) {
                servo.GrabBrick(0.2);
                Switchstate("bitmap");
            }*/
            if (step.equals("bitmap")) {
                switch (bitmap.Skystone(red)) {
                    case "default":
                        // nothing goes here because this get passed to case 3 and 6
                    case "3 & 6":
                        if (!red) {
                            strafe = 0;
                            deliver1stbrick = 48;
                            get2ndbrick = 71;
                            deliver2ndbrick = 70;
                        } else {
                            strafe = 14;
                            deliver1stbrick = 48;
                            get2ndbrick = 69;
                            deliver2ndbrick = 75;
                        }
                        pos3and6 = true;
                        break;
                    case "2 & 5":
                        if (!red) {
                            strafe = 5;
                            deliver1stbrick = 54;
                            get2ndbrick = 76;
                            deliver2ndbrick = 78;
                        } else {
                            strafe = 6;
                            deliver1stbrick = 54;
                            get2ndbrick = 74.5;
                            deliver2ndbrick = 76;
                        }
                        break;
                    case "1 & 4":
                        if (!red) {
                            strafe = 15.5;
                            deliver1stbrick = 60;
                            get2ndbrick = 74;
                            deliver2ndbrick = 81;
                        } else {
                            strafe = 0;
                            deliver1stbrick = 64;
                            get2ndbrick = 78;
                            deliver2ndbrick = 78;
                        }
                        pos1and4 = true;
                        break;
                }
                Switchstate("strafe");
            }
            if (step.equals("strafe")) {
                if (!(strafe == 0)) ;

            }
           /* if (step.equals("turn1")) {
                if (stateTime.milliseconds() > 200) {
                    movement.turnPD(0, .1, .1, 3);
                }
                if (stateTime.milliseconds() > 400) {
                    Switchstate("vision");
                }
            }*/

            if (step.equals("vision")) {
                vision.Update();
                whatToDo.Update(vision.results.left, vision.results.right, vision.results.top, vision.results.bottom, vision.results.confidence, red, firstbrick);
                distance = whatToDo.results.distance;
                if (stateTime.milliseconds() >= 400)
                    Switchstate("forward2");

            }
            /*if (step.equals("downlift1")) {
                movement.liftEncoder(1, -500, 2);
                Switchstate("forward2");
            }*/


            if (step.equals("forward2")) {
                if (!red) {
                    if (firstbrick) {
                        if (!pos1and4) {
                            movement.GyroMoveInch(1, distance + 12, 3);
                        } else {
                            movement.GyroMoveInch(1, distance + 12, 3);
                        }
                    } else {
                        if (!pos1and4) {
                            movement.GyroMoveInch(1, distance + 6, 3);
                        } else {
                            movement.GyroMoveInch(1, 17, 3);
                        }
                    }
                } else {
                    if (firstbrick) {
                        movement.GyroMoveInch(1, distance + 6, 3);
                    } else {
                        if (!pos1and4) {
                            movement.GyroMoveInch(1, distance - 1, 3);
                        } else {
                            movement.GyroMoveInch(1, 21, 3);
                        }
                    }
                }
                Switchstate("grab1");

            }

            if (step.equals("grab1")) {
                mechanisms.GrabBrick(.6);
                if (stateTime.milliseconds() > 300)
                    Switchstate("back1");
            }

            if (step.equals("back1")) {
                if (firstbrick) {
                    if (red) {
                        if (!pos1and4) {
                            movement.MoveInch(-1, 22, 3);
                        } else {
                            movement.MoveInch(-1, 26, 3);
                        }
                    } else {
                        if (!pos1and4) {
                            movement.MoveInch(-1, 26, 3);
                        } else {
                            movement.MoveInch(-1, 19, 3);
                        }
                    }
                } else {
                    if (!pos1and4) {
                        if (red) {
                            movement.MoveInch(-1, 21, 3);
                        } else {
                            movement.MoveInch(-1, 17, 3);
                        }
                    } else {
                        movement.MoveInch(-1, 24, 3);
                    }
                }
                Switchstate("turn2");
            }

            if (step.equals("turn2")) {
                if (stateTime.milliseconds() > 200) {
                    if (!red) {
                        if (!pos1and4) {
                            movement.turnPD(-90, .68, .9, 3);
                        } else {
                            movement.turnPD(-90, .68, .9, 3);
                        }
                    } else {
                        if (!pos1and4) {
                            movement.turnPD(90, .68, .9, 3);
                        } else {
                            movement.turnPD(90, .68, .9, 3);
                        }
                    }
                }
                if (stateTime.milliseconds() > 600) {
                    if (firstbrick) {
                        Switchstate("forward3");
                    } else {
                        Switchstate("forward4");
                    }
                }

            }

            if (step.equals("forward3")) {
                movement.GyroMoveInch(1, deliver1stbrick, 5);
                Switchstate("lift");
            }
            if (step.equals("lift")) {
                movement.liftEncoder(1, 1250, 5);
                movement.GyroMoveInch(1, 16, 1);
                sleep(500);
                movement.liftEncoder(1, -375, 2);
                Switchstate("dropBrick");
            }

            if (step.equals("dropBrick")) {
                mechanisms.GrabBrick(.35);
                if (!red) {
                    movement.gyroInchHeading(-1, 12, 2, -90);
                } else {
                    movement.gyroInchHeading(-1, 12, 2, 90);
                }
                movement.liftEncoder(.6, -875, 3);
                Switchstate("back2");
            }

            if (step.equals("back2")) {
                if (!red) {
                    movement.gyroInchHeading(-1, get2ndbrick + 4, 5, -90);
                } else {
                    movement.gyroInchHeading(-1, get2ndbrick + 4, 5, 90);
                }
                Switchstate("turn3");
                sleep(500);
            }

            if (step.equals("turn3")) {
                if (stateTime.milliseconds() > 200) {
                    if (!pos1and4) {
                        movement.turnPD(0, .68, .9, 3);
                    } else {
                        if (!red) {
                            movement.turnPD(22, .68, .9, 5);
                        } else {
                            movement.turnPD(-28, .68, .9, 3);
                        }
                    }
                }
                firstbrick = false;
                if (stateTime.milliseconds() > 400) {
                    Switchstate("vision");
                }
            }

            if (step.equals("forward4")) {
                if (stateTime.milliseconds() > 200) {
                    if (!red) {
                        movement.gyroInchHeading(1, deliver2ndbrick, 5, -90);
                    } else {
                        movement.GyroMoveInch(1, deliver2ndbrick, 5);
                    }
                    /*if (red) {
                        if (!pos3and6) {
                            Switchstate("dropBrick2");
                        } else {
                            Switchstate("lift1");
                        }
                    } else {
                        Switchstate("lift1");
                    }*/
                    Switchstate("lift1");
                }
            }
            if (step.equals("lift1")) {
                movement.liftEncoder(1, 2000, 5);
                movement.GyroMoveInch(1, 16, 1);
                movement.liftEncoder(1, -400, 5);
                Switchstate("dropBrick2");
            }

            if (step.equals("dropBrick2")) {
                mechanisms.GrabBrick(.35);
                Switchstate("back4");
            }

            if (step.equals("back4")) {
                movement.GyroMoveInch(-1, 32, 3);
                /*if (red && !pos3and6) {
                    break;
                } else {*/
                movement.liftEncoder(1, -1600, 3);
                if (red) {
                    movement.leftGyroStrafe(.25, 6, 4, 90);
                }
                break;

            }


            telemetry.addData("currentstate:", step);
            telemetry.addData("statetime:", stateTime);
            //  telemetry.addData("YawAngle:", movement.getGyroYaw());
            telemetry.update();

        }

    }

    public void QuarryAutoV2(boolean red) throws InterruptedException {
        while (!isStopRequested()) {
            movement.Init(this, false);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        boolean firstbrick = true;
        double strafe = 0;
        double deliver1stbrick = 0;
        double get2ndbrick = 0;
        double deliver2ndbrick = 0;
        boolean pos1and4 = false;
        boolean pos3and6 = false;
        double distance = 0;
        double strafeSpeed = 0.6;
        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData("servopos:", mechanisms.clamp.getPosition());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            if (step.equals("forward1")) {
                if (!red) {
                    movement.GyroMoveInchV2(1, 16, 3);
                } else {
                    movement.leftGyroStrafeV2(.4, 3, 5, 0);
                    sleep(200);
                    movement.turnPD(0, .4, .1, 3);

                    movement.GyroMoveInchV2(1, 15, 3);
                }
                Switchstate("bitmap");
            }
           /* if (step.equals("lift")) {
                movement.liftEncoder(1, 500, 2);
                Switchstate("bitmap");
            }*/
/*
            if (step.equals("clampdown")) {
                servo.GrabBrick(0.2);
                Switchstate("bitmap");
            }*/
            if (step.equals("bitmap")) {
                switch (bitmap.Skystone(red)) {
                    case "default":
                        // nothing goes here because this get passed to case 3 and 6
                    case "3 & 6":
                        if (!red) {
                            strafe = 0;
                            deliver1stbrick = 48;
                            get2ndbrick = 68;
                            deliver2ndbrick = 68;
                        } else {
                            strafe = 7;
                            deliver1stbrick = 43;
                            get2ndbrick = 67;
                            deliver2ndbrick = 75;
                        }
                        pos3and6 = true;
                        break;
                    case "2 & 5":
                        if (!red) {
                            strafe = 3;
                            deliver1stbrick = 54;
                            get2ndbrick = 80;
                            deliver2ndbrick = 78;
                        } else {
                            strafe = 3;
                            deliver1stbrick = 51;
                            get2ndbrick = 78;
                            deliver2ndbrick = 76;
                        }
                        break;
                    case "1 & 4":
                        if (!red) {
                            strafe = 7;
                            deliver1stbrick = 60;
                            get2ndbrick = 71;
                            deliver2ndbrick = 73;
                        } else {
                            strafe = 2;
                            deliver1stbrick = 62;
                            get2ndbrick = 72;
                            deliver2ndbrick = 75;
                        }
                        pos1and4 = true;
                        break;
                }
                Switchstate("strafe");
            }
            if (step.equals("strafe")) {
                if (!(strafe == 0)) {
                    if (red && pos1and4) {
                        movement.leftGyroStrafeV2(.3, strafe, 6, 0);
                    } else {
                        movement.rightGyroStrafeV2(.3, strafe, 6, 0);
                    }
                }

                Switchstate("vision");
            }
           /* if (step.equals("turn1")) {
                if (stateTime.milliseconds() > 200) {
                    movement.turnPD(0, .1, .1, 3);
                }
                if (stateTime.milliseconds() > 400) {
                    Switchstate("vision");
                }
            }*/

            if (step.equals("vision")) {
                vision.Update();
                whatToDo.Update(vision.results.left, vision.results.right, vision.results.top, vision.results.bottom, vision.results.confidence, red, firstbrick);
                distance = whatToDo.results.distance;
                if (stateTime.milliseconds() >= 400)
                    Switchstate("forward2");

            }
        /*    if (step.equals("downlift1")) {
                movement.liftEncoder(1, -500, 2);
                Switchstate("forward2");
            }*/


            if (step.equals("forward2")) {
                if (!red) {
                    if (firstbrick) {
                        if (!pos1and4) {
                            movement.gyroInchHeadingV2(1, distance + 12, 3, 0);
                        } else {
                            movement.gyroInchHeadingV2(1, distance + 12, 3, 0);
                        }
                    } else {
                        if (!pos1and4) {
                            movement.GyroMoveInchV2(1, distance + 6, 3);
                        } else {
                            movement.GyroMoveInchV2(1, 21, 3);
                        }
                    }
                } else {
                    if (firstbrick) {
                        movement.gyroInchHeadingV2(1, distance + 11, 3, 0);
                    } else {
                        if (!pos1and4) {
                            movement.GyroMoveInchV2(1, distance - 1, 3);
                        } else {
                            movement.GyroMoveInchV2(1, 21, 3);
                        }
                    }
                }
                Switchstate("grab1");

            }

            if (step.equals("grab1")) {
                mechanisms.GrabBrick(brickDown);
                if (stateTime.milliseconds() > 300)
                    Switchstate("back1");
            }

            if (step.equals("back1")) {
                if (firstbrick) {
                    if (red) {
                        if (!pos1and4) {
                            movement.GyroMoveInchV2(-1, 20, 3);
                        } else {
                            movement.GyroMoveInchV2(-1, 21, 3);
                        }
                    } else {
                        if (!pos1and4) {
                            movement.GyroMoveInchV2(-1, 23, 3);
                        } else {
                            movement.GyroMoveInchV2(-1, 20, 3);
                        }
                    }
                } else {
                    if (!pos1and4) {
                        if (red) {
                            movement.GyroMoveInchV2(-1, 21, 3);
                        } else {
                            movement.GyroMoveInchV2(-1, 26, 3);
                        }
                    } else {
                        movement.GyroMoveInchV2(-1, 22, 3);
                    }
                }
                Switchstate("turn2");
            }

            if (step.equals("turn2")) {
                if (stateTime.milliseconds() > 200) {
                    if (!red) {
                        if (!pos1and4) {
                            movement.turnPD(-90, turn90p, turn90d, 3);
                        } else {
                            movement.turnPD(-90, turn90p, turn90d, 3);
                        }
                    } else {
                        if (!pos1and4) {
                            movement.turnPD(90, turn90p, turn90d, 3);
                        } else {
                            movement.turnPD(90, turn90p, turn90d, 3);
                        }
                    }
                }
                if (stateTime.milliseconds() > 600) {
                    if (firstbrick) {
                        Switchstate("forward3");
                    } else {
                        Switchstate("forward4");
                    }
                }

            }

            if (step.equals("forward3")) {
                movement.GyroMoveInchV2(1, deliver1stbrick, 5);
                Switchstate("lift");
            }
            if (step.equals("lift")) {
                movement.liftEncoder(1, 1000, 5);
                movement.GyroMoveInchV2(1, 16, 1);
                sleep(500);
                Switchstate("dropBrick");
            }

            if (step.equals("dropBrick")) {
                mechanisms.GrabBrick(brickUp);
                if (!red) {
                    movement.gyroInchHeadingV2(-1, 12, 2, -90);
                } else {
                    movement.gyroInchHeadingV2(-1, 12, 2, 90);
                }
                movement.liftEncoder(1, -1000, 3);
                Switchstate("back2");
            }

            if (step.equals("back2")) {
                if (!red) {
                    movement.gyroInchHeadingV2(-1, get2ndbrick + 4, 5, -90);
                } else {
                    movement.gyroInchHeadingV2(-1, get2ndbrick + 4, 5, 90);
                }
                Switchstate("turn3");
                sleep(500);
            }

            if (step.equals("turn3")) {
                if (stateTime.milliseconds() > 200) {
                    if (!pos1and4) {
                        movement.turnPD(0, turn90p, turn90d, 3);
                    } else {
                        if (!red) {
                            movement.turnPD(35, turn105p, turn105d, 5);
                        } else {
                            movement.turnPD(-35, turn105p, turn105d, 3);
                        }
                    }
                }
                firstbrick = false;
                if (stateTime.milliseconds() > 400) {
                    Switchstate("vision");
                }
            }

            if (step.equals("forward4")) {
                if (stateTime.milliseconds() > 200) {
                    if (!red) {
                        movement.gyroInchHeadingV2(1, deliver2ndbrick, 5, -90);
                    } else {
                        movement.gyroInchHeadingV2(1, deliver2ndbrick, 5, 90);
                    }
                    /*if (red) {
                        if (!pos3and6) {
                            Switchstate("dropBrick2");
                        } else {
                            Switchstate("lift1");
                        }
                    } else {
                        Switchstate("lift1");
                    }*/
                    Switchstate("lift1");
                }
            }
            if (step.equals("lift1")) {
                movement.liftEncoder(1, 1800, 5);
                movement.GyroMoveInchV2(1, 16, 1);
                Switchstate("dropBrick2");
            }

            if (step.equals("dropBrick2")) {
                mechanisms.GrabBrick(brickUp);
                Switchstate("back4");
            }

            if (step.equals("back4")) {
                movement.GyroMoveInchV2(-1, 28, 3);
                /*if (red && !pos3and6) {
                    break;
                } else {*/
                movement.liftEncoder(1, -1800, 3);
                break;

            }


            telemetry.addData("currentstate:", step);
            telemetry.addData("statetime:", stateTime);
            //  telemetry.addData("YawAngle:", movement.getGyroYaw());
            telemetry.update();

        }

    }

    public void FoundationAuto(boolean red, boolean wall) {
        while (!isStopRequested()) {
            movement.Init(this, false);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        step = "strafe1";


        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        stateTime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (step.equals("strafe1")) {
                if (!red) {
                    movement.leftGyroStrafe(.6, 16, 3, 0);
                } else {
                    movement.rightGyroStrafe(.6, 22, 3, 0);
                }
                Switchstate("forward1");
            }


            if (step.equals("forward1")) {
                if (!red) {
                    movement.MoveInch(.6, 38, 5);
                } else {
                    movement.MoveInch(.6, 41, 5);
                }
                if (stateTime.milliseconds() > 1000)
                    Switchstate("clamp1");
            }

            if (step.equals("clamp1")) {
                mechanisms.Foundation(0.4, 0.6);
                Switchstate("back1");
            }
            if (step.equals("back1")) {
                sleep(600);
                movement.GyroMoveInch(-1, 65, 3);
                mechanisms.Foundation(1, 0);
                mechanisms.GrabBrick(.5);
                if (wall) {
                    sleep(18000);
                } else {
                    sleep(7000);
                }
                Switchstate("strafe2");
            }
            if (step.equals("strafe2")) {
                if (wall) {
                    if (!red) {
                        movement.rightGyroStrafe(.6, 23, 5, 0);
                        movement.turnPD(-90, .68, .9, 4);
                        movement.gyroInchHeading(-.4, 36, 4, -90);
                        movement.leftGyroStrafe(.2, 8, 5, -90);
                    } else {
                        movement.leftGyroStrafe(.6, 23, 5, 0);
                        movement.turnPD(90, .68, .9, 4);
                        movement.gyroInchHeading(-.4, 36, 4, 90);
                        movement.rightGyroStrafe(.2, 8, 5, 90);

                    }
                } else {
                    if (!red) {
                        movement.rightGyroStrafe(.6, 30, 5, 0);
                        movement.MoveInch(1, 29, 2);
                        movement.turnPD(-90, .68, .9, 3);
                        movement.MoveInch(-1, 24, 3);
                    } else {
                        movement.leftGyroStrafe(.6, 34, 5, 0);
                        movement.MoveInch(1, 26, 2);
                        movement.turnPD(90, .68, .9, 3);
                        movement.MoveInch(-1, 24, 3);
                    }
                }
                break;
            }
           /* if (step.equals("back1")) {
                movement.GyroMoveInch(-.5, 12, 4);
                if (stateTime.milliseconds() > 300)
                    Switchstate("arcturn1");
            }
            //   if (step.equals("strafe2")) {
            //     movement.Strafe(-strafeSpeed, 8);
            //   if (stateTime.milliseconds() > 300)
            //     Switchstate("arcturn1");
            //  }

            if (step.equals("arcturn1")) {
                if (red) {
                    movement.reversearcturnPD(90, .8, .45, 5);
                } else {
                    movement.reversearcturnPD(-110, .8, .45, 5);
                }
                Switchstate("clamp2");

            }

            if (step.equals("clamp2")) {
                servo.Foundation(1, 0);
                Switchstate("forward2");
            }
            if (step.equals("forward2")) {
                if (!red) {
                    movement.MoveInch(1, 18, 2);
                } else {
                    movement.MoveInch(1, 22, 2);
                }
                Switchstate("strafe3");
            }

            if (step.equals("strafe3")) {
                if (wall) {
                    movement.Strafe(strafeSpeed, 22);
                } else {
                    movement.Strafe(-strafeSpeed, 2);
                }
                Switchstate("turn1");
            }

            if (step.equals("turn1")) {
                if (red) {
                    movement.turnPD(90, .4, .45, 3);
                } else {
                    movement.turnPD(-90, .4, .45, 3);
                }
                if (stateTime.milliseconds() > 600)
                    Switchstate("back2");
            }

            if (step.equals("back2")) {
                if (!red) {
                    movement.MoveInch(-1, 42, 3);
                } else {
                    movement.MoveInch(-1, 48, 3);
                }
                servo.GrabBrick(.9);
                break;
            }*/

            telemetry.addData("currentstate:", step);
            telemetry.addData("statetime:", stateTime);
            telemetry.update();
        }

    }

    public void FoundationAutoV2(boolean red, boolean wall) {
        while (!isStopRequested()) {
            movement.Init(this, true);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        step = "strafe1";


        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        stateTime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (step.equals("strafe1")) {
                if (!red) {
                    movement.leftGyroStrafeV2(.3, 10, 3, 0);
                } else {
                    movement.rightGyroStrafeV2(.3, 18, 3, 0);
                }
                Switchstate("forward1");
            }


            if (step.equals("forward1")) {
                if (!red) {
                    movement.foundationForwardV2(.6, 38, 5, 0);
                } else {
                    movement.foundationForwardV2(.6, 48, 5, 0);
                }
                if (stateTime.milliseconds() > 1000)
                    Switchstate("clamp1");
            }

            if (step.equals("clamp1")) {
                sleep(600);
                mechanisms.Foundation(foundation1down, foundation2down);
                Switchstate("back1");
            }
            if (step.equals("back1")) {
                sleep(600);
                movement.GyroMoveInchV2(-.4, 55, 3);
                mechanisms.Foundation(foundation1up, foundation2up);
                mechanisms.GrabBrick(.5);
                if (wall) {
                    sleep(16000);
                } else {
                    sleep(7000);
                }
                Switchstate("strafe2");
            }
            if (step.equals("strafe2")) {
                if (wall) {
                    if (!red) {
                        movement.rightGyroStrafeV2(.6, 11.5, 5, 0);
                        movement.liftEncoder(1, -1000, 3);
                        movement.turnPD(-90, .68, .9, 4);
                        movement.gyroInchHeadingV2(-.4, 36, 4, -90);
                        movement.leftGyroStrafeV2(.2, 8, 5, -90);
                    } else {
                        movement.leftGyroStrafeV2(.6, 11.5, 5, 0);
                        movement.liftEncoder(1, -1000, 3);
                        movement.turnPD(90, .68, .9, 4);
                        movement.gyroInchHeadingV2(-.4, 36, 4, 90);
                        movement.rightGyroStrafeV2(.2, 4, 5, 90);

                    }
                } else {
                    if (!red) {
                        movement.rightGyroStrafeV2(.6, 30, 5, 0);
                        movement.MoveInchV2(1, 29, 2);
                        movement.turnPD(-90, turn90p, turn90d, 3);
                        movement.MoveInchV2(-1, 24, 3);
                    } else {
                        movement.leftGyroStrafeV2(.6, 34, 5, 0);
                        movement.MoveInchV2(1, 26, 2);
                        movement.turnPD(90, turn90p, turn90d, 3);
                        movement.MoveInchV2(-1, 24, 3);
                    }
                }
                break;
            }
           /* if (step.equals("back1")) {
                movement.GyroMoveInch(-.5, 12, 4);
                if (stateTime.milliseconds() > 300)
                    Switchstate("arcturn1");
            }
            //   if (step.equals("strafe2")) {
            //     movement.Strafe(-strafeSpeed, 8);
            //   if (stateTime.milliseconds() > 300)
            //     Switchstate("arcturn1");
            //  }

            if (step.equals("arcturn1")) {
                if (red) {
                    movement.reversearcturnPD(90, .8, .45, 5);
                } else {
                    movement.reversearcturnPD(-110, .8, .45, 5);
                }
                Switchstate("clamp2");

            }

            if (step.equals("clamp2")) {
                servo.Foundation(1, 0);
                Switchstate("forward2");
            }
            if (step.equals("forward2")) {
                if (!red) {
                    movement.MoveInch(1, 18, 2);
                } else {
                    movement.MoveInch(1, 22, 2);
                }
                Switchstate("strafe3");
            }

            if (step.equals("strafe3")) {
                if (wall) {
                    movement.Strafe(strafeSpeed, 22);
                } else {
                    movement.Strafe(-strafeSpeed, 2);
                }
                Switchstate("turn1");
            }

            if (step.equals("turn1")) {
                if (red) {
                    movement.turnPD(90, .4, .45, 3);
                } else {
                    movement.turnPD(-90, .4, .45, 3);
                }
                if (stateTime.milliseconds() > 600)
                    Switchstate("back2");
            }

            if (step.equals("back2")) {
                if (!red) {
                    movement.MoveInch(-1, 42, 3);
                } else {
                    movement.MoveInch(-1, 48, 3);
                }
                servo.GrabBrick(.9);
                break;
            }*/

            telemetry.addData("currentstate:", step);
            telemetry.addData("statetime:", stateTime);
            telemetry.update();
        }

    }

    public void CarryAuto(boolean red) throws InterruptedException {
        while (!isStopRequested()) {
            movement.Init(this, true);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        step = "forward1";

        boolean firstbrick = true;
        double strafe = 0;
        double deliver1stbrick = 0;
        boolean pos1and4 = false;
        boolean pos3and6 = false;
        double distance = 0;
        double strafeSpeed = .4;
        double grabFoundation = 10;
        if (red)
            strafeSpeed = strafeSpeed * -1;
        double get2ndbrick = 0;
        double deliver2ndbrick = 0;


        stateTime.reset();
        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (step.equals("forward1")) {
                if (red) {
                    movement.Strafe(strafeSpeed, 8);
                }
                movement.GyroMoveInch(1, 16, 3);
                Switchstate("bitmap");
            }

                   /* if (step.equals("lift")) {
                        movement.liftEncoder(1, 500, 2);
                        Switchstate("bitmap");
                    }*/

            if (step.equals("bitmap")) {
                switch (bitmap.Skystone(red)) {
                    case "default":
                        // nothing goes here because this get passed to case 3 and 6
                    case "3 & 6":
                        if (!red) {
                            strafe = 0;
                            deliver1stbrick = 71;
                            get2ndbrick = 73;
                            deliver2ndbrick = 65;
                        } else {
                            strafe = 13;
                            strafeSpeed = strafeSpeed * -1;
                            deliver1stbrick = 73;
                            get2ndbrick = 75;
                            deliver2ndbrick = 67;
                        }
                        pos1and4 = false;
                        pos3and6 = true;
                        break;
                    case "2 & 5":
                        if (!red) {
                            strafe = 8;
                            deliver1stbrick = 81;
                            get2ndbrick = 24;
                        } else {
                            strafe = 8;
                            strafeSpeed = strafeSpeed * -1;
                            deliver1stbrick = 81;
                            get2ndbrick = 24;
                        }
                        pos1and4 = false;
                        pos3and6 = false;
                        break;
                    case "1 & 4":
                        if (!red) {
                            strafe = 10.25;
                            deliver1stbrick = 89;
                            get2ndbrick = 12;
                        } else {
                            strafe = 0;
                            deliver1stbrick = 89;
                            get2ndbrick = 12;
                        }
                        pos1and4 = true;
                        pos3and6 = false;
                        break;
                }
                Switchstate("strafe");
            }
            if (step.equals("strafe")) {
                if (strafe == 0) {
                    Switchstate("vision");
                } else {
                    movement.Strafe(strafeSpeed, strafe);
                    Switchstate("turn1");
                }
            }
            if (step.equals("turn1")) {
                if (stateTime.milliseconds() > 200) {
                    if (firstbrick) {
                        movement.turnPD(0, .2, .2, 3);
                    } else {
                        if (!pos1and4) {
                            movement.turnPD(0, .68, .9, 4);
                        } else {
                            movement.turnPD(45, .68, .9, 4);
                        }
                    }
                }
                if (stateTime.milliseconds() > 400) {
                    Switchstate("vision");
                }
            }

            if (step.equals("vision")) {
                vision.Update();
                whatToDo.Update(vision.results.left, vision.results.right, vision.results.top, vision.results.bottom, vision.results.confidence, red, firstbrick);
                distance = whatToDo.results.distance;
                if (stateTime.milliseconds() >= 400)
                    Switchstate("forward2");

            }
                  /*  if (step.equals("downlift1")) {
                        movement.liftEncoder(1, -500, 2);
                        Switchstate("forward2");
                    }*/


            if (step.equals("forward2")) {
                if (firstbrick) {
                    movement.MoveInch(1, distance + 8, 3);
                } else {
                    if (!pos1and4) {
                        movement.MoveInch(1, distance - 1, 3);
                    } else {
                        movement.MoveInch(1, 23, 3);
                    }
                }
                Switchstate("grab1");

            }

            if (step.equals("grab1")) {
                mechanisms.GrabBrick(.6);
                if (stateTime.milliseconds() > 300)
                    Switchstate("back1");
            }

            if (step.equals("back1")) {
                if (firstbrick) {
                    movement.MoveInch(-1, 14, 3);
                } else {
                    movement.MoveInch(-1, 19, 3);
                }
                Switchstate("turn2");
            }

            if (step.equals("turn2")) {
                if (!red) {
                    movement.turnPD(-90, .68, .9, 4);
                } else {
                    movement.turnPD(90, .68, .9, 4);
                }
                if (firstbrick) {
                    Switchstate("forward3");
                } else {
                    Switchstate("deliver2ndbrick");
                }
            }


            if (step.equals("forward3")) {
                movement.GyroMoveInch(1, deliver1stbrick, 5);
                Switchstate("lift2");
            }
            if (step.equals("lift2")) {
                movement.liftEncoder(1, 1000, 1);
                Switchstate("turn3");
            }
            if (step.equals("turn3")) {
                movement.turnPD(0, .68, .9, 5);
                Switchstate("move2Foundation");
            }

            if (step.equals("move2Foundation")) {
                movement.MoveInch(1, grabFoundation, 4);
                Switchstate("downBrick");
            }
            if (step.equals("downBrick")) {
                movement.liftEncoder(.4, -250, 1);
                Switchstate("BrickGrab");
            }
            if (step.equals("BrickGrab")) {
                mechanisms.Foundation(.5, .7);
                mechanisms.GrabBrick(.65);
                if (stateTime.milliseconds() > 500)
                    Switchstate("back2");
            }
            if (step.equals("back2")) {
                movement.GyroMoveInch(-.5, 12, 1);
                Switchstate("arcturn1");
            }


            if (step.equals("arcturn1")) {
                if (red) {
                    movement.reversearcturnPD(90, 1, .45, 5);
                } else {
                    movement.reversearcturnPD(-110, 1, .45, 5);
                }
                Switchstate("back2");

            }

            if (step.equals("back2")) {
                mechanisms.Foundation(1, 0);
                movement.GyroMoveInch(.5, 32, 4);
                sleep(500);
                movement.MoveInch(-1, 20, 2);
                if (!red) {
                    movement.Strafe(.4, 16);
                    movement.turnPD(-90, .2, .2, 3);
                } else {
                    movement.Strafe(-.4, 16);
                    movement.turnPD(90, .2, .2, 3);
                }
                movement.liftEncoder(1, -750, 1);
                movement.GyroMoveInch(-1, get2ndbrick, 4);
                Switchstate("turn1");
            }
            /*if (step.equals("armsUp")) {
                servo.Foundation(1, 0);
                Switchstate("strafe2");
            }
            if (step.equals("strafe2")) {
                movement.Strafe(strafeSpeed, 26);
                Switchstate("downlift2");
            }
            if (step.equals("downlift2")) {
                movement.liftEncoder(1, -750, 1);
                movement.turnPD(0, .2, .15, 2);
                Switchstate("forward4");
            }
            if (step.equals("forward4")) {
                movement.MoveInch(1, 27, 5);
                Switchstate("turn4");
            }
            if (step.equals("turn4")) {
                if (!red) {
                    movement.turnPD(90, .68, .9, 4);
                } else {
                    movement.turnPD(-90, .68, .9, 4);
                }
                Switchstate("forward5");
            }
            if (step.equals("forward5")) {
                movement.GyroMoveInch(1, get2ndbrick, 3);
                firstbrick = false;
                if (pos3and6) {
                    if (!red) {
                        Switchstate("turn1");
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }*/
            if (step.equals("deliver2ndbrick")) {
                movement.GyroMoveInch(1, deliver2ndbrick, 3);
                Switchstate("34points");
            }
            if (step.equals("34points")) {
                mechanisms.GrabBrick(.65);
                Switchstate("39points");
            }
            if (step.equals("39points")) {
                movement.MoveInch(-1, 30, 3);
                break;
            }


            telemetry.addData("currentstate:", step);
            telemetry.addData("statetime:", stateTime);
            //  telemetry.addData("YawAngle:", movement.getGyroYaw());
            telemetry.update();
        }
    }

    public void TestAny() {
        while (!isStopRequested()) {
            movement.Init(this, false);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        stateTime.reset();
        waitForStart();

        if (step.equals("forward1")) {
            movement.turnPD(90, .5, .6, 10);

        }
    }

    public void Park(boolean far) {
        while (!isStopRequested()) {
            movement.Init(this, false);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        step = "servo";
        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        stateTime.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            if (step.equals("servo")) {
                mechanisms.GrabBrick(.6);
                Switchstate("forward1");
            }
            if (step.equals("forward1")) {
                if (!far) {
                    movement.GyroMoveInch(.4, 24, 5);
                } else {
                    movement.GyroMoveInch(.7, 32, 5);
                }
                break;
            }
        }
    }

    public void ParkV2(boolean far) {
        while (!isStopRequested()) {
            movement.Init(this, false);
            vision.Init(this);
            mechanisms.Init(this);
            bitmap.Init(this);
            whatToDo.Init(this);
            break;
        }
        step = "servo";
        /** Wait for the game to begin */
        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        stateTime.reset();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            if (step.equals("servo")) {
                mechanisms.GrabBrick(.6);
                Switchstate("forward1");
            }
            if (step.equals("forward1")) {
                if (!far) {
                    movement.GyroMoveInchV2(.4, 24, 5);
                } else {
                    movement.GyroMoveInchV2(.7, 32, 5);
                }
                break;
            }
        }
    }

    public void Switchstate(String newState) {
        step = newState;
        stateTime.reset();
        int soundID = hardwareMap.appContext.getResources().getIdentifier(newState, "raw", hardwareMap.appContext.getPackageName());
        if (soundID != 0)
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
    }

}



