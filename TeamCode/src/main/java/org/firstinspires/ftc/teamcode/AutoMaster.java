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
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "AutoMaster", group = "godlyAuto")
@Disabled
public abstract class AutoMaster extends LinearOpMode {


//    private WhatToDo whatToDo = new WhatToDo();
    //    public Vision vision = new Vision();
  /*
    public Movement movement = new Movement();
    public Mechanisms mechanisms = new Mechanisms();
    public RingDetect bitmap = new RingDetect();
*/
    public Mechanisms mechanisms = new Mechanisms();
    public Movement movement = new Movement();
    public RingDetect bitmap;

    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime stateTime = new ElapsedTime();

    public int voltage = 1;

    double lifterUp = .6429;
    double lifterDown = 1;
    double anglerPower = .735;
    double anglerDown = 0;
    double anglerUp = .75;

    String step = "forward1";

    public void TestAny() throws InterruptedException {
        while (!isStopRequested()) {
            movement.Init(this);
            //    vision.Init(this);
            bitmap = new RingDetect(this);
            mechanisms.Init(this);
            break;
        }
        /** Wait for the game to begin */

        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData("voltage:", voltage);
        telemetry.addData(">", "Press the Dpad to assign power from voltage.");
        telemetry.update();
        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad2.dpad_up) {
                voltage += 1;
                while (gamepad2.dpad_up);
            }
            if (gamepad2.dpad_down) {
                voltage -= 1;
                while (gamepad2.dpad_down);
            }
            telemetry.addData("Stage 1:", "<= 13.8 (if under 13.6, yike)");
            telemetry.addData("Stage 2:", "<= 14 and >13.8");
            telemetry.addData("Stage 3:", "<= 14.1 and >14");
            telemetry.addData("Stage 4:", ">14.1");
            telemetry.addData("CurrentStage:", voltage);
            telemetry.update();
            idle();
        }

        while (opModeIsActive() && !isStopRequested()) {
       //  movement.rightGyroStrafe(1, 48, 3,0);
            movement.turnPD(180, .73, .98, 5);
          //  mechanisms.powerShot();
            //   movement.moveInch(1,48,10);
            //   movement.gyroInch(.5,73,100,0);
            break;
        }

    }



    public void BlueAuto() throws InterruptedException {
        while (!isStopRequested()) {
            movement.Init(this);
        //    vision.Init(this);
            bitmap = new RingDetect(this);
            mechanisms.Init(this);
            break;
        }


        telemetry.addData("YawAngle:", movement.getGyroYaw());
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        stateTime.reset();
        while (!isStopRequested() && !opModeIsActive()) {
            if (gamepad2.dpad_up) {
                voltage += 1;
                while (gamepad2.dpad_up) ;
            }
            if (gamepad2.dpad_down) {
                voltage -= 1;
                while (gamepad2.dpad_down) ;
            }
            telemetry.addData("Stage 1:", "<= 13.8 (if under 13.6, yike)");
            telemetry.addData("Stage 2:", "<= 14 and >13.8");
            telemetry.addData("Stage 3:", "<= 14.1 and >14");
            telemetry.addData("Stage 4:", ">14.1");
            telemetry.addData("CurrentStage:", voltage);
            telemetry.update();
            idle();
        }

        while (opModeIsActive() && !isStopRequested()) {
            //  mechanisms.armWobble(false);
          //  String Vision = bitmap.getWobble();
            String Vision = "A";
         //   movement.rightGyroStrafe(1, 13 - 1, 3, 0);
            sleep(250);
            while (!isStopRequested() && opModeIsActive()) {
                switch (Vision) {

                    case "A":
                        movement.gyroEncoderInch(-1, 68, 5, 0);
                        mechanisms.armWobble(true, false, false);
                        movement.gyroEncoderInch(1, 12, 2, 0);
                        movement.leftGyroStrafe(.6, 28, 3, 0);
                        sleep(250);
                        break;

                    case "B":
                        movement.gyroEncoderInch(-1, 92, 7, 0);
                        movement.turnPD(90, .75, .3, 3);
                        movement.gyroEncoderInch(-1, 12, 1, 90);
                        mechanisms.armWobble(true, false, false);
                        movement.gyroEncoderInch(1, 12, 1, 90);
                        movement.turnPD(0, .7, 4, 3);
                        movement.gyroEncoderInch(1, 36, 5, 0);
                        sleep(250);
                        movement.leftGyroStrafe(1, 26, 3, 0);
                        sleep(250);
                        break;

                    case "C":
                        movement.gyroEncoderInch(-1, 112, 5, 0);
                        mechanisms.armWobble(true, false, false);
                        movement.gyroEncoderInch(1, 55, 3, 0);
                        sleep(250);
                        movement.leftGyroStrafe(1, 27, 3, 0);
                        sleep(250);


                        //      movement.moveInch(-1, 48, 4);

                }
                break;
            }

            movement.turnPD(180, .6, .98, 3);
            mechanisms.powerShot();
            while (!isStopRequested() && opModeIsActive()) {
                switch (Vision) {
                    case "A":
                        movement.gyroEncoderInch(-1, 37, 4, 180);
                        movement.leftGyroStrafe(1, 13, 3, 180);
                        mechanisms.armWobble(false, false, true);
                        movement.rightGyroStrafe(1, 10, 3, 180);
                        movement.gyroEncoderInch(1, 56, 4, 180);
                        movement.turnPD(-90, .7, .3, 3);
                        movement.gyroEncoderInch(-1, 38, 4, -90);
                        mechanisms.armWobble(true, false, false);
                        sleep(250);
                        movement.gyroEncoderInch(1,20,1,-90);
                        movement.turnPD(180,.7,.3,3);
                        break;

                    case "B":
                        movement.gyroEncoderInch(-1, 41, 4, 180);
                        movement.leftGyroStrafe(1, 12.5, 3, 180);
                        mechanisms.armWobble(false, false, true);
                        movement.rightGyroStrafe(1, 6, 3, 180);
                        movement.gyroEncoderInch(1, 62, 4, 180);
                        movement.turnPD(-25, .9, .5, 4);
                        movement.gyroEncoderInch(-1, 13, 1, -25);
                        mechanisms.armWobble(true, false, false);
                        movement.gyroEncoderInch(1, 12, 1, -25);
                        break;

                    case "C":
                        movement.gyroEncoderInch(-1, 38., 4, 180);
                        movement.leftGyroStrafe(1, 12.8, 3, 180);
                        mechanisms.armWobble(false, false, true);
                        movement.leftGyroStrafe(1, 20, 3, 180);
                        movement.turnPD(0, .6, .98, 4);
                        movement.gyroEncoderInch(-1, 91, 3, 0);
                        mechanisms.armWobble(true, false, false);
                        movement.gyroEncoderInch(1, 37, 3, 0);
                }

                //     movement.gyroInch(1, 20, 3, 180);
                break;
            }
            break;
        }
    }
           // movement.moveInch(1,34, 2);


    public void Switchstate(String newState) {
        step = newState;
        stateTime.reset();
        int soundID = hardwareMap.appContext.getResources().getIdentifier(newState, "raw", hardwareMap.appContext.getPackageName());
        if (soundID != 0)
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
    }

}



