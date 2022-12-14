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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Objects;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Tin Diesel Auto Code Red", group = "Auto")
public class TinDieselAutoCode extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated mode'l
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */

    public final static double SERVO_MIN_RANGE = -1.0;
    public final static double SERVO_MAX_RANGE = 1.0;


    private static final String TFOD_MODEL_ASSET = "dieselVision2.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "C",
            "GN",
            "VD"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vufvuforiaoria.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    private static final String VUFORIA_KEY =
            "AdmkB9//////AAABmbaqp4qRNkhehSXxBNkbbuUC0ks4mF8T6061vE4LQk3Qup1AXLlHJ6Q0MPzUdA1wq6RHQuedh8NraZFkz9OvTd8inlTADDC1JC/vgiqqyFkOSrOz+zeWOMCqE1Uru/Sye+8NCmZx70enSS09nE7/uuP2zD9rDIHnCL7NSthXkWum7HxaxoDdzgitkqKbcgoj2AAPnvDFmXrrbfe9ZRLdsxFMXfWg9kmFP9KTSGJd9PxV0liTO29+PqADyUw31LsH1duWx1oMfXDObbtcy9qar8lcOWqmOCqLVakspphxmgDqNOlha5w/I8WqdZX7/2dw+dcb35o2tqr/4XDUAVVR34SRJzGcCkceES2k4Qt5aRM5";

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

    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        DcMotor lFD = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor lBD = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rFD = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor rBD = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor aS = hardwareMap.get(DcMotor.class, "arm_slider");

        Servo aG = hardwareMap.get(Servo.class, "arm_grabber");

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            if(Objects.equals(recognition.getLabel(), "VD")){

                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(1500);
                                lBD.setPower(0);
                                lFD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(60000);

                                /*

                                aG.setPosition(SERVO_MAX_RANGE);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(150);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(2000); //need to check time
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(500);
                                lFD.setPower(0.1);
                                lBD.setPower(0.1);
                                rFD.setPower(0.1);
                                rBD.setPower(0.1);
                                sleep(250);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(500);
                                aS.setPower(0.8); //check power and direction for arm slider
                                sleep(1000); //check time for arm slider to reach for grabber
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(500);
                                lFD.setPower(0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.5);
                                rBD.setPower(-0.5);
                                sleep(200);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                aG.setPosition(SERVO_MIN_RANGE); //this could be the wrong range have to check
                                sleep(500); //need to check the time for servo to open
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(200);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                aS.setPower(-0.8);
                                sleep(1000);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(200);
                                lFD.setPower(-0.1);
                                lBD.setPower(-0.1);
                                rFD.setPower(-0.1);
                                rBD.setPower(-0.1);
                                sleep(500);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(300);
                                lFD.setPower(0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.5);
                                rBD.setPower(-0.5);
                                sleep(1000);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(60000);

                                 */
                            }
                            else if(Objects.equals(recognition.getLabel(), "GN")){

                            lFD.setPower(-0.5);
                            lBD.setPower(-0.75);
                            rFD.setPower(0.5);
                            rBD.setPower(0.5);
                            sleep(900);;
                            lFD.setPower(0);
                            lBD.setPower(0);
                            rFD.setPower(0);
                            rBD.setPower(0);
                            aS.setPower(0);
                            sleep(500);
                            lFD.setPower(-0.5);
                            lBD.setPower(0.5);
                            rFD.setPower(-0.5);
                            rBD.setPower(0.5);
                            sleep(1500);
                            lBD.setPower(0);
                            lFD.setPower(0);
                            rFD.setPower(0);
                            rBD.setPower(0);
                            sleep(60000);

                                /*

                                aG.setPosition(SERVO_MAX_RANGE);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(150);
                                aS.setPower(-0.25);
                                sleep(100);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(200);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(2300); //need to check time
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(500);
                                lFD.setPower(0.1);
                                lBD.setPower(0.1);
                                rFD.setPower(0.1);
                                rBD.setPower(0.1);
                                sleep(150);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(500);
                                aS.setPower(-0.25); //check power and direction for arm slider
                                sleep(400); //check time for arm slider to reach for grabber
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(500);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(200);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                aG.setPosition(SERVO_MIN_RANGE); //this could be the wrong range have to check
                                sleep(500); //need to check the time for servo to open
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                lFD.setPower(0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.5);
                                rBD.setPower(-0.5);
                                sleep(200);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                aS.setPower(0.25);
                                sleep(400);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(200);
                                lFD.setPower(-0.1);
                                lBD.setPower(-0.1);
                                rFD.setPower(-0.1);
                                rBD.setPower(-0.1);
                                sleep(150);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(300);
                                lFD.setPower(0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.5);
                                rBD.setPower(-0.5);
                                sleep(1000);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(300);
                                lFD.setPower(0.75);
                                lBD.setPower(0.75);
                                rFD.setPower(-0.5);
                                rBD.setPower(-0.5);
                                sleep(1000);
                                lBD.setPower(0);
                                lFD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(60000);

                                 */
                            }
                            else if(Objects.equals(recognition.getLabel(), "C")){

                                lFD.setPower(0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.75);
                                rBD.setPower(-0.75);
                                sleep(750);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(500);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(1500);
                                lBD.setPower(0);
                                lFD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(60000);

                                /*


                                aG.setPosition(SERVO_MAX_RANGE);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(150);
                                aS.setPower(-0.25);
                                sleep(100);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(200);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(2300); //need to check time
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(500);
                                lFD.setPower(0.1);
                                lBD.setPower(0.1);
                                rFD.setPower(0.1);
                                rBD.setPower(0.1);
                                sleep(150);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(500);
                                aS.setPower(-0.25); //check power and direction for arm slider
                                sleep(400); //check time for arm slider to reach for grabber
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(500);
                                lFD.setPower(-0.5);
                                lBD.setPower(0.5);
                                rFD.setPower(-0.5);
                                rBD.setPower(0.5);
                                sleep(200);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                aG.setPosition(SERVO_MIN_RANGE); //this could be the wrong range have to check
                                sleep(500); //need to check the time for servo to open
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                lFD.setPower(0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.5);
                                rBD.setPower(-0.5);
                                sleep(200);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(300);
                                aS.setPower(0.25);
                                sleep(400);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                aS.setPower(0);
                                sleep(200);
                                lFD.setPower(-0.1);
                                lBD.setPower(-0.1);
                                rFD.setPower(-0.1);
                                rBD.setPower(-0.1);
                                sleep(150);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(300);
                                lFD.setPower(0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.5);
                                rBD.setPower(-0.5);
                                sleep(1000);
                                lFD.setPower(0);
                                lBD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(300);
                                lFD.setPower(-0.5);
                                lBD.setPower(-0.5);
                                rFD.setPower(0.75);
                                rBD.setPower(0.75);
                                sleep(1000);
                                lBD.setPower(0);
                                lFD.setPower(0);
                                rFD.setPower(0);
                                rBD.setPower(0);
                                sleep(60000);

                                 */
                            }

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                        }
                        telemetry.update();
                    }

                    else{

                        lFD.setPower(-0.5);
                        lBD.setPower(-0.75);
                        rFD.setPower(0.5);
                        rBD.setPower(0.5);
                        sleep(900);;
                        lFD.setPower(0);
                        lBD.setPower(0);
                        rFD.setPower(0);
                        rBD.setPower(0);
                        aS.setPower(0);
                        sleep(500);
                        lFD.setPower(-0.5);
                        lBD.setPower(0.5);
                        rFD.setPower(-0.5);
                        rBD.setPower(0.5);
                        sleep(1500);
                        lBD.setPower(0);
                        lFD.setPower(0);
                        rFD.setPower(0);
                        rBD.setPower(0);
                        sleep(60000);
                        /*
                        lFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rBD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        lFD.setTargetPosition(-2000);
                        lBD.setTargetPosition(2000);
                        rFD.setTargetPosition(-2000);
                        rBD.setTargetPosition(2000);
                        sleep(2000);
                        lFD.setTargetPosition(0);
                        lBD.setTargetPosition(0);
                        rFD.setTargetPosition(0);
                        rBD.setTargetPosition(0);
                        sleep(1000);
                        lFD.setTargetPosition(1000);
                        lBD.setTargetPosition(1000);
                        rFD.setTargetPosition(-1000);
                        rBD.setTargetPosition(-1000);
                        sleep(1000);
                        lFD.setTargetPosition(0);
                        lBD.setTargetPosition(0);
                        rFD.setTargetPosition(0);
                        rBD.setTargetPosition(0);
                        sleep(60000);



                        lFD.setPower(-0.5);
                        lBD.setPower(0.5);
                        rFD.setPower(-0.5);
                        rBD.setPower(0.5);
                        sleep(1800);
                        lFD.setPower(0);
                        lBD.setPower(0);
                        rFD.setPower(0);
                        rBD.setPower(0);
                        sleep(1000);
                        lFD.setPower(0.75);
                        lBD.setPower(0.75);
                        rFD.setPower(-0.5);
                        rBD.setPower(-0.5);
                        sleep(1000);
                        lFD.setPower(0);
                        lBD.setPower(0);
                        rFD.setPower(0);
                        rBD.setPower(0);
                        sleep(60000);
                         */
                    }

                }
            }
        }
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "12414camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
