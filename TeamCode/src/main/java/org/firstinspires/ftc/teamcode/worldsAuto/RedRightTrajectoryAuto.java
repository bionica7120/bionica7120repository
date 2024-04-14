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

package org.firstinspires.ftc.teamcode.worldsAuto;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.hardwaremap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RedTrajectoryAuto - RIGHT")
//@Disabled
public class RedRightTrajectoryAuto extends LinearOpMode {

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
    public void runOpMode() throws InterruptedException {


        robot = new hardwaremap();
        robot.init(hardwareMap);

        initTfod();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory center1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(29)
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .back(6)
                .build();

        Trajectory center3 = drive.trajectoryBuilder(center2.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .back(28)
                .build();

        Trajectory center4 = drive.trajectoryBuilder(center3.end())
                .strafeRight(5)
                .build();

        Trajectory center5 = drive.trajectoryBuilder(center4.end())
                .back(7)
                .build();

        Trajectory center6 = drive.trajectoryBuilder(center5.end())
                .forward(2)
                .build();

        Trajectory center7 = drive.trajectoryBuilder(center6.end())
                .strafeLeft(18)
                .build();

        Trajectory left1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(31)
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeLeft(2)
                .build();

        Trajectory left3 = drive.trajectoryBuilder(left2.end())
                .back(37)
                .build();

        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .strafeRight(8)
                .build();

        Trajectory left5 = drive.trajectoryBuilder(left4.end())
                .forward(3)
                .build();

        Trajectory left6 = drive.trajectoryBuilder(left5.end())
                .strafeLeft(28)
                .build();

        Trajectory right1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(31)
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .strafeRight(10)
                .build();

        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .forward(28)
                .build();

        Trajectory right4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0, 0, Math.toRadians(180))), false)
                .back(4)
                .build();

        Trajectory right5 = drive.trajectoryBuilder(right4.end())
                .strafeLeft(14)
                .build();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();



        if (opModeIsActive()) {

            intakeWristDown();
            intakeOpenOrClose(-0.5, 250);

            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                if (pixelDetected) {

                    intakeWristUp();

                    if (0 <= x && x <= 213)
                    {
                        condition = 2;
                        telemetry.addLine("Pixel detected");
                        telemetry.addData("Condition", condition);
                        telemetry.update();

                        drive.followTrajectory(left1);
                        sleep(500);
                        drive.turn(Math.toRadians(90));
                        sleep(500);
                        intakeOpenOrClose(0.5, 1000);
                        sleep(500);
                        drive.followTrajectory(left2);
                        sleep(500);
                        drive.followTrajectory(left3);
                        sleep(500);
                        drive.followTrajectory(left4);
                        sleep(500);
                        //drive.turn(Math.toRadians(180));
                        //sleep(500);
                        moveArm(-0.8, 1000);
                        sleep(500);
                        intakeOpenOrClose(0.5, 1000);
                        sleep(500);
                        moveArm(0.5, 1000);
                        moveArm(-0.2, 250);
                        sleep(500);
                        drive.followTrajectory(left5);
                        sleep(500);
                        drive.followTrajectory(left6);

                        break;


                    }

                    if (x >= 426)
                    {
                        condition = 3;
                        telemetry.addLine("Pixel detected");
                        telemetry.addData("Condition", condition);
                        telemetry.update();

                        drive.followTrajectory(right1);
                        sleep(500);
                        drive.turn(Math.toRadians(-90));
                        sleep(500);
                        intakeOpenOrClose(0.5, 1000);
                        sleep(500);
                        drive.followTrajectory(right2);
                        sleep(500);
                        drive.followTrajectory(right3);
                        sleep(500);
                        drive.turn(Math.toRadians(180));
                        sleep(500);
                        drive.followTrajectory(right4);
                        sleep(500);
                        moveArm(-0.8, 1000);
                        sleep(500);
                        intakeOpenOrClose(0.5, 1000);
                        sleep(500);
                        moveArm(0.5, 1000);
                        moveArm(-0.2, 250);
                        sleep(500);
                        drive.followTrajectory(right5);

                        break;

                    }

                    if (condition == 1) {
                        telemetry.addLine("Pixel detected");
                        telemetry.addData("Condition", condition);
                        telemetry.update();

                        drive.followTrajectory(center1);
                        sleep(500);
                        intakeOpenOrClose(0.5, 1000);
                        sleep(500);
                        drive.followTrajectory(center2);
                        sleep(500);
                        drive.turn(Math.toRadians(90));
                        sleep(500);
                        drive.followTrajectory(center3);
                        sleep(500);
                        drive.followTrajectory(center4);
                        sleep(500);
                        drive.followTrajectory(center5);
                        sleep(500);
                        intakeWristDown();
                        moveArm(-0.8, 1000);
                        sleep(500);
                        intakeOpenOrClose(0.5, 1000);
                        sleep(500);
                        moveArm(0.5, 1000);
                        moveArm(-0.2, 250);
                        sleep(500);
                        drive.followTrajectory(center6);
                        sleep(500);
                        drive.followTrajectory(center7);



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

    public void intakeWristUp() {
        robot.intakeWrist.setPosition(1);
    }

    public void intakeWristDown() {
        robot.intakeWrist.setPosition(0);
    }

    public void moveArm (double power, int milliseconds) throws InterruptedException {
        robot.arm.setPower(power);
        sleep(milliseconds);
        robot.arm.setPower(0);
    }

}   // end class