package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous (name = "CompTensorAuto")

public class CompTensorAuto extends LinearOpMode {
    private CompBotHardwareMap robot;
    private Recognition recognition;

    private final double circumference = Math.PI * 2.95;
    private final double ticks = 560;

    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CenterStage.tflite";
    private static final String[] LABELS = {
            "Pixel"
    };

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    public boolean pixelDetected;
    public int condition = 1;


    @Override
    public void runOpMode() {
        robot = new CompBotHardwareMap();
        robot.init(hardwareMap);
        initTfod();

        robot.init(hardwareMap);

        // Wait for the game to begin

        while (opModeInInit()) {
            telemetry.addData(">", "Press Play to start op mode");
            // Push telemetry to the Driver Station.
            telemetry.update();
        }

        if (isStopRequested()) return;
        waitForStart();
        telemetry.clearAll();

        drive(0.2);
        sleep(4350);
        drive(0);
        sleep(1000);


        telemetryTfod();
        if (pixelDetected) {
            telemetry.addLine("Pixel detected");
            telemetry.addData("Condition", condition);
            telemetry.update();
            drive(0.2);
            sleep(1200);
            drive(0);
            sleep(1000);
            intakeOpenOrClose(0.8, 1000);
        } else {
            condition++;
                telemetry.addLine("Pixel not detected");
                telemetry.addData("Condition", condition);
                telemetry.update();
                driveRot(0.5);
                sleep(1000);
                driveLeftOrRight(-0.2);
                sleep(200);
                drive(0);
                sleep(1500);
                telemetryTfod();
                if (pixelDetected)
                {
                    telemetry.addLine("Pixel detected");
                    telemetry.addData("Condition", condition);
                    telemetry.update();
                    drive(0.2);
                    sleep(1200);
                    drive(0);
                    sleep(1000);
                    intakeOpenOrClose(0.8, 1000);
                }



        }


        if (condition == 1) {
            drive(-0.2);
            sleep(3500);
            drive(0);
            sleep(1000);
            driveLeftOrRight(-0.2);
            sleep(7000);
            drive(0);
        } else if (condition == 2) {
//            drive(-0.2);
//            sleep(3500);
//            drive(0);
//            sleep(500);
//            driveRot(500);
//            sleep(100);
//            drive(0);
//            sleep(500);
//            drive(0.2);
//            sleep(2000);

        }


    }




//Backdrop code
//        if (condition == 1)
//        {
//            driveRotate(0.2, 20);
//            driveForward(0.2, 10);
//        } else if (condition == 2)
//        {
//            driveRotate(0.2, 40);
//            driveForward(0.2, 10);
//        } else if (condition == 3)
//        {
//            driveForward(0.2, 10);
//        }




        //main();
     // end of runOpMode()

    //public abstract void main();

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
/*
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.

 */
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                /*
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
*/
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
/*
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
*/
        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
/*
        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);
*/
    }   // end method initTfod()

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addLine();

            if (currentRecognitions.size() > 0)
            {
                pixelDetected = true;
            }

        }   // end for() loop

        if (currentRecognitions.size() == 0)
        {
            pixelDetected = false;
        }

    }   // end method telemetryTfod()

    public void driveForward(double power, double distance) {
        //reset encoders
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rotations needed given a distance
        double rotationsNeeded = distance / circumference;

        //ticks
        int encoderTarget = (int) (rotationsNeeded * ticks);


        //set target position of encoders
        robot.leftFrontDrive.setTargetPosition(encoderTarget);
        robot.leftBackDrive.setTargetPosition(encoderTarget);
        robot.rightFrontDrive.setTargetPosition(encoderTarget);
        robot.rightBackDrive.setTargetPosition(encoderTarget);

        //to reverse, set encoder target to negative
        //to strafe set 2 motors to negative encoder target

        //set power
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);

//        for (DcMotor motor : robot.drivetrain) {
//            motor.setPower(power);
//        }


        //run to position
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftFrontDrive.isBusy()) {

        }


    }


    public void intakeOpenOrClose(double power, int milliseconds) {
        robot.intake.setPower(power);
        sleep(milliseconds);
        robot.intake.setPower(0);
    }

    public void driveStrafeRightorLeft(double power, double distance) {
        //reset encoders
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rotations needed given a distance
        double rotationsNeeded = distance / circumference;

        //ticks
        int encoderTarget = (int) (rotationsNeeded * ticks);

        //set target position
        robot.leftFrontDrive.setTargetPosition(-encoderTarget);
        robot.leftBackDrive.setTargetPosition(encoderTarget);
        robot.rightFrontDrive.setTargetPosition(encoderTarget);
        robot.rightBackDrive.setTargetPosition(-encoderTarget);


        //set power
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);

//        for (DcMotor motor : robot.drivetrain) {
//            motor.setPower(power);
//        }

        //run to position
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftFrontDrive.isBusy()) {

        }
    }

    public void drive(double power){
        robot.leftFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
    }

    public void driveRotate(double power, double distance){
        //reset encoders
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rotations needed given a distance
        double rotationsNeeded = distance / circumference;

        //ticks
        int encoderTarget = (int)(rotationsNeeded * ticks);

        //set target position
        robot.leftBackDrive.setTargetPosition(encoderTarget);
        robot.rightBackDrive.setTargetPosition(-encoderTarget);
        robot.leftFrontDrive.setTargetPosition(encoderTarget);
        robot.rightFrontDrive.setTargetPosition(-encoderTarget);

        //to reverse, set encoder target to negative
        //to strafe set 2 motors to negative encoder target

        //set power
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);

//        for (DcMotor motor : robot.drivetrain) {
//            motor.setPower(power);
//        }

        //run to position
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.leftFrontDrive.isBusy()) {

        }

    }

    public void driveRot(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
    }

    public void moveArmUp(double power, double distance) {
        //reset encoders
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rotations needed given a distance
        double rotationsNeeded = distance / circumference;

        //ticks
        int encoderTarget = (int) (rotationsNeeded * ticks);


        //set target position of encoders

        robot.arm.setTargetPosition(encoderTarget);

        //to reverse, set encoder target to negative
        //to strafe set 2 motors to negative encoder target

        //set power

        robot.arm.setPower(power);

//        for (DcMotor motor : robot.drivetrain) {
//            motor.setPower(power);
//        }


        //run to position
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.arm.isBusy()) {

        }


    }

    public void moveArmDown(double power, double distance) {
        //reset encoders
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rotations needed given a distance
        double rotationsNeeded = distance / circumference;

        //ticks
        int encoderTarget = (int) (rotationsNeeded * ticks);


        //set target position of encoders

        robot.arm.setTargetPosition(-encoderTarget);

        //to reverse, set encoder target to negative
        //to strafe set 2 motors to negative encoder target

        //set power

        robot.arm.setPower(power);

//        for (DcMotor motor : robot.drivetrain) {
//            motor.setPower(power);
//        }


        //run to position
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.arm.isBusy()) {

        }


    }

    public void driveLeftOrRight(double power) {

        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);
    }
}
