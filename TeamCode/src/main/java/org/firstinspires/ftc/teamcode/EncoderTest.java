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
import com.qualcomm.robotcore.hardware.DcMotor;

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
@Autonomous(name = "EncoderTest")
//@Disabled
public class EncoderTest extends LinearOpMode {

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

    private final double circumference = Math.PI * 2.95;
    private final double ticks = 560; //560

    public boolean pixelDetected;
    public int condition = 1;
    public double x;
    public double y;

    @Override
    public void runOpMode() {

        robot = new hardwaremap();
        robot.init(hardwareMap);




        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //rotations needed given a distance
                double rotationsNeeded = 10 / circumference;

                //ticks
                int encoderTarget = (int) (rotationsNeeded * ticks);


                //set target position of encoders
                robot.leftFrontDrive.setTargetPosition(encoderTarget);

                robot.leftFrontDrive.setPower(0.5);
                robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Encoder Count", robot.leftFrontDrive.getTargetPosition());
                // Push telemetry to the Driver Station.
                telemetry.update();
                return;



//    public void driveForward(double power, double distance) {
//        //reset encoders
//        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //rotations needed given a distance
////        double rotationsNeeded = distance / circumference;
////
////        //ticks
////        int encoderTarget = (int) (rotationsNeeded * ticks);
//
//
//        //set target position of encoders
//        robot.leftFrontDrive.setTargetPosition(encoderTarget);
//        robot.leftBackDrive.setTargetPosition(encoderTarget);
//        robot.rightFrontDrive.setTargetPosition(encoderTarget);
//        robot.rightBackDrive.setTargetPosition(encoderTarget);
//
//        //to reverse, set encoder target to negative
//        //to strafe set 2 motors to negative encoder target
//
//        //set power
//        robot.leftFrontDrive.setPower(power);
//        robot.leftBackDrive.setPower(power);
//        robot.rightFrontDrive.setPower(power);
//        robot.rightBackDrive.setPower(power);
//
////        for (DcMotor motor : robot.drivetrain) {
////            motor.setPower(power);
////        }
//
//
//        //run to position
//        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy()) {
//
//        }
//
//
//    }
//
//    public void driveStrafeRightorLeft(double power, double distance) {
//        //reset encoders
//        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //rotations needed given a distance
//        double rotationsNeeded = distance / circumference;
//
//        //ticks
//        int encoderTarget = (int) (rotationsNeeded * ticks);
//
//        //set target position
//        robot.leftFrontDrive.setTargetPosition(-encoderTarget);
//        robot.leftBackDrive.setTargetPosition(encoderTarget);
//        robot.rightFrontDrive.setTargetPosition(encoderTarget);
//        robot.rightBackDrive.setTargetPosition(-encoderTarget);
//
//
//        //set power
//        robot.leftFrontDrive.setPower(power);
//        robot.leftBackDrive.setPower(power);
//        robot.rightFrontDrive.setPower(power);
//        robot.rightBackDrive.setPower(power);
//
////        for (DcMotor motor : robot.drivetrain) {
////            motor.setPower(power);
////        }
//
//        //run to position
//        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy()) {
//
//        }
//    }
//
//    public void driveRotate(double power, double distance){
//        //reset encoders
//        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //rotations needed given a distance
//        double rotationsNeeded = distance / circumference;
//
//        //ticks
//        int encoderTarget = (int)(rotationsNeeded * ticks);
//
//        //set target position
//        robot.leftBackDrive.setTargetPosition(encoderTarget);
//        robot.rightBackDrive.setTargetPosition(-encoderTarget);
//        robot.leftFrontDrive.setTargetPosition(encoderTarget);
//        robot.rightFrontDrive.setTargetPosition(-encoderTarget);
//
//        //to reverse, set encoder target to negative
//        //to strafe set 2 motors to negative encoder target
//
//        //set power
//        robot.leftBackDrive.setPower(power);
//        robot.rightBackDrive.setPower(power);
//        robot.leftFrontDrive.setPower(power);
//        robot.rightFrontDrive.setPower(power);
//
////        for (DcMotor motor : robot.drivetrain) {
////            motor.setPower(power);
////        }
//
//        //run to position
//        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy()) {
//
//        }
//
//    }
//
//    public void moveArm(double power, double distance) {
//        //reset encoders
//        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        //rotations needed given a distance
//        double rotationsNeeded = distance / circumference;
//
//        //ticks
//        int encoderTarget = (int) (rotationsNeeded * ticks);
//
//
//        //set target position of encoders
//
//        robot.arm.setTargetPosition(encoderTarget);
//
//        //to reverse, set encoder target to negative
//        //to strafe set 2 motors to negative encoder target
//
//        //set power
//
//        robot.arm.setPower(power);
//
////        for (DcMotor motor : robot.drivetrain) {
////            motor.setPower(power);
////        }
//
//
//        //run to position
//        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (robot.arm.isBusy()) {
//
//        }
//
//
//    }



}}}}   // end class
