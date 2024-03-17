package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class TeleOpFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    ScaleInputDrive(-gamepad1.left_stick_y),
                    ScaleInputDrive(-gamepad1.left_stick_x)
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            ScaleInputDrive(-gamepad1.right_stick_x)
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public float ScaleInputDrive(float ScaleInputDrive) {
        // This is an array with the numbers that you can see.  This will allow both joystick values
        // to be close(not having to be exact) and still be able to get the same power as the other
        // if needed, or if not needed, it can help the driver to be a little more exact while driving.
        double[] DriveArray = {0,.1,.15,.2,.25,.3,.4,.45,.5,.6,.7,.75,.8, .85,.9,1};
        // For every number in this DriveArray, make sure you have the proper number below
        //ScaleInputDrive is multiplied by (DriveArray total variables) 15 because arrays start at 0, so its numbers 0-15, instead of 1-16
        int DriveIndex = (int) (ScaleInputDrive * 15);
        // This allows for "negative" numbers in the array, without having to directly enter them
        if (DriveIndex < 0) {
            DriveIndex = -DriveIndex;
        }

        double DriveScale = 0;
        if (ScaleInputDrive < 0) {
            DriveScale = -DriveArray[DriveIndex];
        } else {
            DriveScale = DriveArray[DriveIndex];
        }
        // Returns the value DriveScale, which is used with the Joysticks when they are set to
        // the variables DriveLeft and DriveRight
        return (float)DriveScale;
    }
}