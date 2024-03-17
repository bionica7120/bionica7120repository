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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "CompRedPropAuto - TAPE LEFT")
//@Disabled
public class CompRedPropAutoTapeLeft extends LinearOpMode {

    private hardwaremap robot;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedPropModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "7120",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public boolean pixelDetected;
    public int condition = 1;
    double x;
    double y;

    @Override
    public void runOpMode() {

        robot = new hardwaremap();
        robot.init(hardwareMap);
        initTfod();

        robot.init(hardwareMap);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                if (pixelDetected) {

                    if (0 <= x && x <= 213)
                    {
                        condition = 2;
                        telemetry.addLine("Pixel detected");
                        telemetry.addData("Condition", condition);
                        telemetry.update();

                        drive(0.2);
                        sleep(4700);
                        drive(0);
                        sleep(1000);

                        drive(0);
                        sleep(1000);

                        driveRot(0.5);
                        sleep(900);

                        drive(0);
                        sleep(1000);

//                        drive(0.2);
//                        sleep(500);

                        drive(0);
                        sleep(1000);

                        intakeOpenOrClose(1, 3000);

                        drive(0);
                        sleep(1000);

                        drive(-0.2);
                        sleep(500);

                        drive(0);
//                        sleep(1000);
//
//                        driveRot(0.5);
//                        sleep(1900);
//
//                        drive(0);
//                        sleep(1000);
//
//                        drive(0.2);
//                        sleep(5000);
//
//                        drive(0);

                        break;


                    }

                    if (x >= 426)
                    {
                        condition = 3;
                        telemetry.addLine("Pixel detected");
                        telemetry.addData("Condition", condition);
                        telemetry.update();

                        drive(0.2);
                        sleep(4700);
                        drive(0);
                        sleep(1000);

                        drive(0);
                        sleep(1000);

                        driveRot(-0.5);
                        sleep(900);

                        drive(0);
                        sleep(1000);

                        intakeOpenOrClose(1, 3000);

                        drive(0);
                        sleep(1000);

                        drive(-0.2);
                        sleep(200);

                        drive(0);
//                        sleep(1000);
//
//                        driveRot(-0.5);
//                        sleep(900);
//
//                        drive(0);
//                        sleep(1000);
//
//                        drive(0.2);
//                        sleep(2000);
//
//                        drive(0);
//                        sleep(1000);
//
//                        driveRot(0.5);
//                        sleep(900);

//                        drive(0);
//                        sleep(1000);
//
//                        drive(0.2);
//                        sleep(8000);
//
//                        //stop
//                        drive(0);

                        break;

                    }

                    if (condition == 1) {
                        telemetry.addLine("Pixel detected");
                        telemetry.addData("Condition", condition);
                        telemetry.update();

                        drive(0.2);
                        sleep(4700);
                        drive(0);
                        sleep(1000);

                        //forward
//                        drive(0.2);
//                        sleep(1100);

                        //stop
                        drive(0);
                        sleep(1000);

                        //deposits pixel
                        intakeOpenOrClose(1, 3000);

                        //stop
                        drive(0);
                        sleep(1000);

                        //backward
                        drive(-0.2);
                        sleep(500);

                        //stop
                        drive(0);
//                        sleep(1000);
//
//                        //rotate
//                        driveRot(-0.5);
//                        sleep(900);
//
//                        //stop
//                        drive(0);
//                        sleep(1000);
//
//                        //forward
//                        drive(0.2);
//                        sleep(6000);
//
//                        //stop
//                        drive(0);
//                        sleep(1000);
//
//                        //rotate
//                        driveRot(0.5);
//                        sleep(1800);
//
//                        //stop
//                        drive(0);
//                        sleep(1000);
//
//                        //arm up
//                    moveArm(-0.7);
//                    sleep(800);
//
//                    drive(-0.3);
//                    sleep(200);
//
//
//                    intakeOpenOrClose(1, 5000);

                        break;


                    }
                }

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (currentRecognitions.size() > 0)
            {
                pixelDetected = true;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    public void drive(double power){
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
    }

    public void driveRot(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    public void intakeOpenOrClose(double power, int milliseconds) {
        robot.intake.setPower(power);
        sleep(milliseconds);
        robot.intake.setPower(0);
    }

    public void moveArm (double power) {
        robot.arm.setPower(power);
    }

}   // end class