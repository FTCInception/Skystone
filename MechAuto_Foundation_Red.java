/* Copyright (c) 2017 FIRST. All rights reserved.
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

package Inception.Skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * This file houses Autonomous code
 * It uses the main robot class Refbot to define the hardware and driving commands
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *
 *  See the Refbot class for encode-based driving controls that perform the actual movement.
 *
 */
@Autonomous(name="Mech: Auto Foundation Red", group="MechBot")
public class MechAuto_Foundation_Red extends LinearOpMode {

    /* Declare OpMode members. */
    private MechBot         robot   = new MechBot();   // Use a Pushbot's hardware

    private static final double     DRIVE_SPEED             = 0.4;
    private static final double     TURN_SPEED              = 0.4;
    private static final double     PIVOT_SPEED             = 0.6;
    private static final double     SQ                      = 70/3.0;        // Length of 3 squares / 3 in case we want to think that way
    private String className = this.getClass().getSimpleName().toLowerCase();
    private double P=0.05;
    private double a=0;

    @Override
    public void runOpMode() {
        double turnDirection;
        double d[] = {0.3, 0.4};

        // Red or blue alliance -- only difference is the turn direction
        if (className.contains("blue")) {
            turnDirection = 1.0;
        } else {
            turnDirection = -1.0;
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initAutonomous(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
/*
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * -90,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * 90,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * -180,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * 180,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * -270,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * 270,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * -360,30);
        robot.fastEncoderRotate(TURN_SPEED, turnDirection * 360,30);

        a = robot.fastEncoderStraight(DRIVE_SPEED,12,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-12,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,24,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-24,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,48,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-48,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,72,30, P);
        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-72,30, P);
        robot.straightA = a;
 */

        robot.releaseFoundation(0);

        a = robot.fastEncoderStraight(DRIVE_SPEED,-14,2);

        a = robot.gyroRotate(TURN_SPEED, (turnDirection * 90)-a,2);

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-18,2);

        a = robot.gyroRotate(TURN_SPEED, (turnDirection * -90)-a,2);

        robot.straightA = a;
        a = robot.fastEncoderStraight(DRIVE_SPEED,-19,2);

        robot.grabFoundation(1500);

        robot.straightA = a;
        a = robot.fastEncoderDrive(DRIVE_SPEED,18, 18, 3, 0.05, d, d );

        a = robot.gyroPivot(PIVOT_SPEED, (turnDirection * 110)-a, 4);

        robot.releaseFoundation(750);

        robot.fastEncoderDrive(DRIVE_SPEED,4, 4, 1, 0.05, d, d );

        robot.grabFoundation(1000);

        // FIXME -- Need to record final angle here and use it to make sure we wind up pointed to the wall in the final rotate.
        // FIXME -- This is different than other turns where we are tryign to fix previous error, this angle shoudl be used
        // FIXME -- to guarantee a final heading, not a correction.
        robot.fastEncoderDrive(DRIVE_SPEED,-26, -26,3, 0.0, d, d );

        robot.gyroRotate(TURN_SPEED, (turnDirection * -10), 2);

        robot.fastEncoderStraight(DRIVE_SPEED,35,15, 0.05);
    }
}