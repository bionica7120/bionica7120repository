package org.firstinspires.ftc.teamcode.parkingonly;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CompBotHardwareMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous (name = "RedRightBackAuto")

public class RedRightBackAuto extends LinearOpMode {
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
        //moveArm(0.8, 500);
//        drive(0.2);
//        sleep(500);
//        drive(0);
//        sleep(1000);
//        driveLeftOrRight(-0.2);
//        sleep(10000);
//        drive(0);
//        intakeOpenOrClose(1, 3000);

        drive(0.5);
        sleep(200);
        drive(0);
        sleep(1000);
        driveRot(0.5);
        sleep(1000);
        drive(0);
        sleep(1000);
        drive(-0.5);
        sleep(2800);
        drive(0);
        intakeOpenOrClose(1, 2000);




//        while (opModeIsActive())
//        {
//            telemetryTfod();
//            if (pixelDetected) {
//                telemetry.addLine("Pixel detected");
//                telemetry.update();
//                drive(0.2);
//                sleep(1200);
//                drive(0);
//                sleep(1000);
//                intakeOpenOrClose(0.8, 1000);
//                break;
//            } else {
//                //condition++;
//                telemetry.addLine("Pixel not detected");
//
//                // driveForward(0.2, 6);
//                // driveRotate(-0.2, 20);
////                    if (condition == 3) {
////                        break;
////                    }
//            }
//
//            telemetry.addData("Condition", condition);
//            telemetry.update();
//        }
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

    public void driveLeftOrRight(double power) {

        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);
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

    public void moveArm(double power, int milliseconds)
    {
        robot.arm.setPower(-power);
        sleep(milliseconds);
        //robot.arm.setPower(0);
    }

    public void driveRot(double power) {
        robot.leftFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.rightBackDrive.setPower(power);
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
}
