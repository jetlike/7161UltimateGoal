package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;

public class WhatToDo {

    String queuedSound = "";
    private AutoMaster auto = null;


    public class Results {
        void Reset() {
            strafe = 0;
            forward = 0;
            grab = false;
        }

        double strafe;
        double forward;
        double distance;
        boolean grab;

        void CopyFrom(Results other) {
            strafe = other.strafe;
            forward = other.forward;
            grab = other.grab;
        }

        boolean IsSame(Results other) {
            if (other.strafe != strafe)
                return false;
            if (other.forward != forward)
                return false;
            return other.grab == grab;
        }

        void DebugWithSound() {
            if (grab)
                queuedSound = "loot";
            else if (forward != 0) {
                if (forward < .4) {
                    queuedSound = "forward_slowly";
                } else {
                    queuedSound = "move_forward";
                }
            } else if (strafe != 0) {
                if (strafe < 0) {
                    queuedSound = "strafe_left";
                }
                if (strafe > 0) {
                    queuedSound = "strafe_right";
                }
            } else {
                queuedSound = "searching";
            }
        }
    }

    public Results results = new Results();
    public Results lastResults = new Results();

    public void Init(AutoMaster autoMaster) {
        auto = autoMaster;
    }

    void Update(double left, double right, double top, double bottom, double confidence, boolean red, boolean firstbrick) {

        results.Reset();
        if (confidence > .5) {
         /*   double middleAvg = (right + left) / 2.0;

            results.strafe = CalcStrafe(middleAvg);*/
            double height = top - bottom;
            if (!red) {
                if (firstbrick) {
                    if (CalcDist(height) > 17 && CalcDist(height) < 19) {
                        results.distance = CalcDist(height);
                        auto.telemetry.addData("accuratVisionDist:", results.distance);
                    } else {
                        results.distance = 18;
                        auto.telemetry.addData("notaccurate:", 18);
                    }
                    auto.telemetry.addData("height:", height);
                } else {
                    if (CalcDist(height) > 17 && CalcDist(height) < 19) {
                        results.distance = CalcDist(height);
                        auto.telemetry.addData("accuratVisionDist:", results.distance);
                    } else {
                        results.distance = 18;
                        auto.telemetry.addData("notaccurate:", 18);
                    }
                    auto.telemetry.addData("height:", height);
                }
            } else {
                if (firstbrick) {
                    if (CalcDist(height) > 18 && CalcDist(height) < 20) {
                        results.distance = CalcDist(height);
                        auto.telemetry.addData("accuratVisionDist:", results.distance);
                    } else {
                        results.distance = 19;
                        auto.telemetry.addData("notaccurate:", 19);
                    }
                    auto.telemetry.addData("height:", height);
                } else {
                    if (CalcDist(height) > 19 && CalcDist(height) < 21) {
                        results.distance = CalcDist(height);
                        auto.telemetry.addData("accuratVisionDist:", results.distance);
                    } else {
                        results.distance = 20;
                        auto.telemetry.addData("notaccurate:", 18);
                    }
                    auto.telemetry.addData("height:", height);
                }
            }
        } else {
            if (!red) {
                results.distance = 18;
                String height = "guessing";
                auto.telemetry.addData("height:", height);
            } else {
                results.distance = 19.5;
                String height = "guessing";
                auto.telemetry.addData("height:", height);
            }
        }


           /* if (results.strafe == 0) {
                results.forward = CalcForward(height);

                if (results.forward == 0) {
                    results.grab = true;
                }
            }
        }

        if (!results.IsSame(lastResults))
        {
            lastResults.CopyFrom(results);
            results.DebugWithSound();
        }*/
        auto.telemetry.addData("distance:", results.distance);
        auto.telemetry.addData("confidence:", confidence);


    }


    double CalcDist(double height) {
        return 6.6 / height;
    }

    double ReturnCent(double distance) {
        return .5;
    }

    private double CalcForward(double height) {

        double speed = 0;
        if (height > .8)
            speed = 0;
        else if (height > .6)
            speed = .2;
        else if (height <= .6)
            speed = .5;

        return speed;

    }

    private double CalcStrafe(double middleAvg) {
        double strafeSpeed = 0.0;
        if (middleAvg < .4) {
            strafeSpeed = -1;
        } else if (middleAvg > .6) {
            strafeSpeed = 1;
        } else {
            strafeSpeed = 0.0;
        }
        return strafeSpeed;
    }

}
