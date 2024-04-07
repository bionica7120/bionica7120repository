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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
@Autonomous(name = "CompRedPropAuto - LEFT ENC")
//@Disabled
public class NewStateMachineRedAuto extends LinearOpMode {
    hardwaremap robot = new hardwaremap();
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "RedPropModel.tflite";
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
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

    private final double circumference = Math.PI * 2.95;
    private final double ticks = 560;

    public boolean pixelDetected;
    public int condition = 1;
    public double x = -1;
    public double y = -1;

    public enum RobotState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
        //states for every starting position - repeat for parking/scoring
        //RL_START
        RR_START
        //RR_DRIVE_TO_SPIKE
        //DEPOSIT
        //RR_DRIVE_TO_BACKDROP
        //ARM_UP
        //DEPOSIT
        //ARM_DOWN
        //PARK
        //BL_START
        //BL_DRIVE_TO_SPIKE
        //DEPOSIT
        //BL_DRIVE_TO_BACKDROP
        //ARM_UP
        //DEPOSIT
        //ARM_DOWN
        //PARK
        //BR_START
        //
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    NewStateMachineRedAuto.RobotState robotState = NewStateMachineRedAuto.RobotState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()

    // used with the dump servo, this will get covered in a bit
    ElapsedTime timer = new ElapsedTime();


    final double DISTANCE_TO_SPIKE = 20;
    final boolean TURN_RIGHT = false;
    final boolean TURN_LEFT = false;
    final double DISTANCE_TO_BACKDROP = 40;
    final double DISTANCE_TO_PARK = 10;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initTfod();
        robot.init(hardwareMap);
        timer.reset();



        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (opModeInInit())
        {

            telemetryTfod();
            if (0 <= x && x <= 213)
            {
                condition = 2;
            } else if (x >= 426) {
                condition = 3;
            } else {
                condition = 1;
            }

            telemetry.addData("Condition", condition);
            telemetry.update();
        }
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (condition == 2)
                {
                    telemetry.addLine("Pixel detected");
                    telemetry.addData("Condition", condition);
                    telemetry.update();

                    break;

                }

                if (condition == 3)
                {
                    telemetry.addLine("Pixel detected");
                    telemetry.addData("Condition", condition);
                    telemetry.update();

                    break;

                }

                if (condition == 1) {

                    telemetry.addLine("Pixel detected");
                    telemetry.addData("Condition", condition);
                    telemetry.update();
                    break;

                }

                robot.leftFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);

                switch (robotState) {
                    case RR_START:



                        break;
                    case LIFT_START:
                        // Waiting for some input
                        if (gamepad1.x) {
                            // x is pressed, start extending
//                            robot.leftFrontDrive.setTargetPosition(LIFT_HIGH);
//                            robotState = StateMachineTensor.RobotState.LIFT_EXTEND;
                        }
                        break;
                    case LIFT_EXTEND:
                        // check if the lift has finished extending,
                        // otherwise do nothing.
//                        if (Math.abs(liftMotor.getCurrentPosition() - LIFT_HIGH) < 10) {
//                            // our threshold is within
//                            // 10 encoder ticks of our target.
//                            // this is pretty arbitrary, and would have to be
//                            // tweaked for each robot.
//
//                            // set the lift dump to dump
//                            //liftDump.setTargetPosition(DUMP_DEPOSIT);
//
//                            timer.reset();
//                            robotState = StateMachineTensor.RobotState.LIFT_DUMP;
//                        }
                        break;
                    case LIFT_DUMP:
//                        if (timer.seconds() >= DUMP_TIME) {
//                            // The robot waited long enough, time to start
//                            // retracting the lift
//                            //liftDump.setTargetPosition(DUMP_IDLE);
//                            liftMotor.setTargetPosition(LIFT_LOW);
//                            robotState = StateMachineTensor.RobotState.LIFT_RETRACT;
//                        }
                        break;
                    case LIFT_RETRACT:
//                        if (Math.abs(liftMotor.getCurrentPosition() - LIFT_LOW) < 10) {
//                            robotState = StateMachineTensor.RobotState.LIFT_START;
//                        }
                        break;
                    default:
                        // should never be reached, as liftState should never be null
                        robotState = NewStateMachineRedAuto.RobotState.LIFT_START;
                }

                // small optimization, instead of repeating ourselves in each
                // lift state case besides LIFT_START for the cancel action,
                // it's just handled here
                if (gamepad1.y && robotState != NewStateMachineRedAuto.RobotState.LIFT_START) {
                    robotState = NewStateMachineRedAuto.RobotState.LIFT_START;
                }

                // mecanum drive code goes here
                // But since none of the stuff in the switch case stops
                // the robot, this will always run!
                //updateDrive(gamepad1, gamepad2);

                // Share the CPU.
                sleep(20);
            }



            }



        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }
    // end runOpMode()

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
        //telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (currentRecognitions.size() > 0)
            {
                pixelDetected = true;
            }

//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()



}   // end class
