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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CloseableFrame;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.nio.ByteBuffer;
import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 *         // This code implements a soft start and soft stop based on time/speed/distance
 *         // It's likely mor ecomplicated than necessary.  A distance=only approach is
 *         // better below.  Turns still seem to have some issue in the time/speed/distance.
 *         static final double     SECONDS_TO_FULL_POWER   = 1.5;
 *         static final double     SPEED_RAMP_PER_MS       = (1.0/1000.0) / SECONDS_TO_FULL_POWER;
 *         static final double     SPEED_OFFS              = 0.05;
 *         double lastPos=0;
 *         double lt;
 *         double rem;
 *         double rate;
 *         boolean decel = false;
 *         ElapsedTime     looptime = new ElapsedTime();
 *         looptime.reset()
 *                 if(false) {
 *                     // Record elapsed time
 *                     lt = looptime.milliseconds();
 *                     // Reset the timer for next loop
 *                     looptime.reset();
 *                     // Get current position
 *                     curPos = Math.abs(robot.leftFDrive.getCurrentPosition());
 *                     // How much farther?
 *                     toGo = tgtPos - curPos;
 *                     // Compute remaining time based on encoder rate from last loop
 *                     rate = ((curPos - lastPos) / lt);
 *                     rem = toGo / rate;
 *                     // Save the new position
 *                     lastPos = curPos;
 *                     // If the delta to our target stop speed is > than our deceleration rate, start to slow down
 *                     if ((actSpeed > 0.20) && ((actSpeed - 0.20) > (rem * SPEED_RAMP_PER_MS))) {
 *                         actSpeed = Math.max(actSpeed - (lt * SPEED_RAMP_PER_MS), 0.20);
 *                         robot.leftFDrive.setPower(actSpeed);
 *                         robot.rightFDrive.setPower(actSpeed * (67 / 70.0));
 *                         robot.leftBDrive.setPower(actSpeed);
 *                         robot.rightBDrive.setPower(actSpeed * (67 / 70.0));
 *                         decel = true;
 *                     } else {
 *                         if (!decel && (actSpeed < speed)) {
 *                             // If our speed is below our target and we're not slowing down, speed up
 *                             actSpeed = Math.min(actSpeed + (lt * SPEED_RAMP_PER_MS), speed);
 *                             robot.leftFDrive.setPower(actSpeed);
 *                             robot.rightFDrive.setPower(actSpeed * (67 / 70.0));
 *                             robot.leftBDrive.setPower(actSpeed);
 *                             robot.rightBDrive.setPower(actSpeed * (67 / 70.0));
 *                         }
 *                     }
 *                 }
 *                 //telemetry.addData("Path3",  "Running rate: %3.3f, rem: %7.0f, s: %1.3f", rate, rem, actSpeed);
 */

@Autonomous(name="Refbot: Auto Drive By Encoder", group="Refbot")
public class RefbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private Refbot          robot   = new Refbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;         // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 10.0/11.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;       // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                              (WHEEL_DIAMETER_INCHES * 3.14159);
    private static final double     AXLE_LENGTH             = 13.25;        // Width of robot through the pivot point (center wheels)
    private static final double     INCHES_PER_DEGREE       = (AXLE_LENGTH * 3.14159) / 360.0;
    private static final double     DRIVE_SPEED             = 0.65;
    private static final double     TURN_SPEED              = 0.30;
    private static final double     SOFT_D                  = 50.0;
    private static final double     KpL                     = 1.0;
    private static final double     KpR                     = 67.0/70.0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at LF:%7d, RF:%7d, LB:%7d, RB:%7d",
        //                  robot.leftFDrive.getCurrentPosition(),
        //                  robot.rightFDrive.getCurrentPosition(),
        //                  robot.leftBDrive.getCurrentPosition(),
        //                  robot.rightBDrive.getCurrentPosition());
        //telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while(true) {
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, 2, 2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
            encoderDrive(DRIVE_SPEED, -2, -2, 10.0);    // S1: Forward 72 Inches
            if (CheckForSkyStone(tfod)!=0) {
                sleep(5000);
            }
        }
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /*
        encoderDrive(DRIVE_SPEED,  72,  72, 10.0);    // S1: Forward 72 Inches
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S2: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S3: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S4: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderRotate(TURN_SPEED,  360, 10.0);                    // S5: Turn Right 1 rotations
        sleep(500);     // pause for servos to move
        encoderDrive(DRIVE_SPEED,  -48,  -48, 10.0);  // S6: Backwards 48 Inches
        */

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        //sleep(10000);     // pause for servos to move

        //telemetry.addData("Path", "Complete");
        //telemetry.update();
    }

    public void encoderRotate(double speed,
                              double degrees,
                              double timeoutS) {
        encoderDrive( speed, degrees * INCHES_PER_DEGREE, -degrees * INCHES_PER_DEGREE, timeoutS);

    }


     /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFTarget;
        int newRightFTarget;
        int newLeftBTarget;
        int newRightBTarget;
        double curPos;
        double tgtPos;
        double toGo;
        double actSpeed=0.0;
        double newSpeed=0.0;
        double spdUp,spdDn;
        double[] speedRamp = {0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0};


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightFTarget = (int)(rightInches * COUNTS_PER_INCH);
            newLeftBTarget = (int)(leftInches * COUNTS_PER_INCH);
            newRightBTarget = (int)(rightInches * COUNTS_PER_INCH);
            tgtPos = Math.abs(newLeftFTarget);
            robot.leftFDrive.setTargetPosition(newLeftFTarget);
            robot.rightFDrive.setTargetPosition(newRightFTarget);
            robot.leftBDrive.setTargetPosition(newLeftBTarget);
            robot.rightBDrive.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion at the minimum speed.
            runtime.reset();
            speed = Math.abs(speed);
            actSpeed = speedRamp[0];
            robot.leftFDrive.setPower(actSpeed * KpL);
            robot.rightFDrive.setPower(actSpeed * KpR);
            robot.leftBDrive.setPower(actSpeed * KpL);
            robot.rightBDrive.setPower(actSpeed * KpR);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFDrive.isBusy() && robot.rightFDrive.isBusy() && robot.leftBDrive.isBusy() && robot.rightBDrive.isBusy())) {


                // This code implements a soft start and soft stop.
                // Get current position
                curPos = Math.abs(robot.leftFDrive.getCurrentPosition());

                // How much farther?
                toGo  = Math.max(0,(tgtPos - curPos));

                // Compute speed on acceleration and deceleration legs
                spdUp = Math.min(speedRamp[Math.min((int)(curPos/SOFT_D),speedRamp.length-1)],speed);
                spdDn = Math.min(speedRamp[Math.min((int)(toGo/SOFT_D),speedRamp.length-1)],speed);

                // Use the minimum speed
                newSpeed = Math.min(spdUp, spdDn);

                // Special case when we get really close, go back to full power and let the PID
                // motor controller handle it
                if (toGo < (SOFT_D - 5)) { newSpeed = speed; }

                // Change power if necessary
                if (newSpeed != actSpeed) {
                    actSpeed = newSpeed;
                    robot.leftFDrive.setPower(actSpeed * KpL);
                    robot.rightFDrive.setPower(actSpeed * KpR);
                    robot.leftBDrive.setPower(actSpeed * KpL);
                    robot.rightBDrive.setPower(actSpeed * KpR);
                }

                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to LF:%7d, RF: %7d, LB: %7d, RB: %7d", newLeftFTarget,  newRightFTarget, newLeftBTarget,  newRightBTarget);
                //telemetry.addData("Path2",  "Running at LF:%7d, RF: %7d, LB: %7d, RB: %7d",
                //                            robot.leftFDrive.getCurrentPosition(),
                //                            robot.rightFDrive.getCurrentPosition(),
                //                            robot.leftBDrive.getCurrentPosition(),
                //                            robot.rightBDrive.getCurrentPosition());
                //telemetry.addData("Path3",  "Running up: %1.3f, dn: %1.3f, s: %1.3f, p: %5d, t: %5d", spdUp, spdDn, actSpeed, (int)curPos, (int)toGo);
                //telemetry.update();
                //sleep(10);   // optional pause after each move
            }

            // Stop all motion;
            robot.leftFDrive.setPower(0);
            robot.rightFDrive.setPower(0);
            robot.leftBDrive.setPower(0);
            robot.rightBDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(500);   // optional pause after each move
        }
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AfWZ0Nj/////AAABma9i7nGZSk81hrDHleShtMuKJES27HbNIQandd3JejLnjvR3256AZU4KbwLKM3zRbhT54zvMHzIwofU7N0TwRifRjMB9sPJ/GZoVpvrcOTNl0F3G6ynufbSkLWWRAGzf3ffMAWeB97a8iF/fPSC5kYY7u56rj2IXVXw7zB2GrTIlFIgkGmy+faJST+4838yCmE4kZFqSc8qnKW1zG0qh9EhMdg8KobZkODSkG2r2uDHXEcvnD8zLKQMIZGm3ueWs1aWvJRZZgx6wDFr1LFnnzZDdJ1en1TjkVWt7Mv+pb8j+9j/9W7Fp4Q5yUrqDl64aeNe7pLplamMYlZXBSOmevv/4r+h6SdQKeimUeP5dCZ6m";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private int CheckForSkyStone(TFObjectDetector tfod) {
        boolean beyondBoundry = false;
        Boolean edgeFound = false;
        int column=0;
        int center=0;

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    for(int j=0; j<1000;j++) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            Recognition skystone_rec = null;
                            for (Recognition recognition : updatedRecognitions) {
                        /*telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        //below will give the left, top, right, bottom but it is disabled because of unnesary feedback
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());*/

                                if (recognition.getLabel().contentEquals("Skystone")) {
                                    skystone_rec = recognition;
                                }
                            }

                            // get Frame
                            CloseableFrame myFrame = vuforia.getFrameQueue().poll();

                            if ((myFrame != null) && (skystone_rec != null)) {
                                Image myImage = myFrame.getImage(0);

                                ByteBuffer myBuffer = myImage.getPixels();
                                int bytes_per_pixel = myImage.getStride() / myImage.getBufferWidth();
                                int row = ((int) (skystone_rec.getTop()) + (int) (skystone_rec.getBottom())) / 4;

                                if (row < 0) {
                                    row = 0;
                                }

                                if (row > (myImage.getBufferHeight() - 1)) {
                                    row = myImage.getBufferHeight() - 1;
                                }

                                column = (int) skystone_rec.getLeft() / 2;
                                int boundry = (int) skystone_rec.getRight() / 2;
                                center = (int) myImage.getBufferWidth() / 2;
                                int step = 4;
                                int windowwidth = 16;
                                float threshold = 0x50 * windowwidth;
                                int PixelIndex = row * myImage.getBufferWidth() + column;

                                telemetry.addData("Feedback", "Row %d", row);

                                telemetry.addData("Feedback", "Buffer height and width %d %d", myImage.getBufferHeight(), myImage.getBufferWidth());

                                telemetry.addData("Feedback", "Pixel bytes %d", bytes_per_pixel);

                                while (!edgeFound) {
                                    int sum = 0;

                                    for (int index = 0; index < windowwidth; index++) {
                                        sum += myBuffer.getChar(PixelIndex + index) & 0x00ff;
                                    }
                                    PixelIndex += step;
                                    column += step;
                                    if (column > boundry) {
                                        edgeFound = true;
                                        beyondBoundry = true;

                                        telemetry.addData("Error", "code is looking for skystone past boundry");
                                    }
                                    if (sum <= threshold) {
                                        edgeFound = true;
                                    }
                                    if (edgeFound == true && beyondBoundry == false) {
                                        telemetry.addData("sum, Pixel Index, column, found edge, offset from center", "%d %d %d %b %d", sum, PixelIndex, column, edgeFound, column - center);
                                        telemetry.update();
                                        int offset = (column - center);
                                        return(offset);
                                        //if ((offset > 50) && (offset < 250)) {
                                        //    return (1);
                                        //}
                                        //if ((offset > -180) && (offset < -60)) {
                                        //    return (2);
                                        //}
                                        //if ((offset > -280) && (offset < -190)) {
                                        //    return (3);
                                        //}
                                    }

                                }

                            } else if (myFrame == null) {
                                telemetry.addData("Note", "No Frame");
                            }
                            telemetry.update();
                        }
                    }
                }

        //if (tfod != null) {
        //    tfod.shutdown();
        //}
        return(0);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();



        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Get Frame
        vuforia.setFrameQueueCapacity(1);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
