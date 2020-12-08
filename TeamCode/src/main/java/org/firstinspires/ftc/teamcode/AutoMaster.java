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

    public void TestAny() {
        while (!isStopRequested()) {
            movement.Init(this);
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


    public void Switchstate(String newState) {
        step = newState;
        stateTime.reset();
        int soundID = hardwareMap.appContext.getResources().getIdentifier(newState, "raw", hardwareMap.appContext.getPackageName());
        if (soundID != 0)
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
    }

}



