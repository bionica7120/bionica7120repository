package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backwards               Left-joystick Forward/Backwards
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backwards when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp
//@Disabled
public class NewRobotCentricCompBot extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private hardwaremap robot;

    public final double circumference = Math.PI * 2.95;
    public final double ticks = 560;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        robot = new hardwaremap();
        robot.init(hardwareMap);

        //double armPower = 0;
        double intakePower = 0;


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = ScaleInputDrive(-gamepad1.left_stick_y);  // Note: pushing stick forward gives negative value
            double lateral =  ScaleInputDrive(gamepad1.left_stick_x);
            double yaw     =  ScaleInputDrive(gamepad1.right_stick_x);



            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            //max = Math.max(max, Math.abs(armPower));
            //max = Math.max(max, Math.abs(intakePower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;

            }

//
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);


            //robot.arm.setPower(Range.clip(gamepad2.right_stick_y, -0.8, 0.8));
            robot.intake.setPower(Range.clip(gamepad2.left_stick_y, -1, 1));

            if (gamepad1.right_bumper)
            {
                robot.suspension.setPower(1);
            }

            if (gamepad1.left_bumper)
            {
                robot.suspension.setPower(-1);
            }

            if (gamepad2.left_bumper)
            {
                robot.arm.setPower(0.5);
            }

            if (gamepad2.right_bumper)
            {
                robot.arm.setPower(-0.5);
            }

            if (!gamepad1.left_bumper && !gamepad1.right_bumper)
            {
                robot.suspension.setPower(0);
            }

            if (!gamepad2.left_bumper && !gamepad2.right_bumper)
            {
                robot.arm.setPower(0);
            }

            if (gamepad2.x)
            {
                robot.drone.setPosition(0);
            }

            if (gamepad2.y)
            {
                robot.drone.setPosition(1);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Power", "RightFrontDrive: " + rightFrontPower);
            telemetry.addData("Power", "RightBackDrive: " + rightBackPower);
            telemetry.addData("Power", "LeftFrontDrive: " + leftFrontPower);
            telemetry.addData("Power", "LeftBackDrive: " + leftBackPower);
            telemetry.addData("Arm Power", robot.arm.getPower());
            telemetry.addData("Intake Power", robot.intake.getPower());
            telemetry.addData("Suspension Power", robot.suspension.getPower());
            telemetry.addData("Drone Position: ", robot.drone.getPosition());
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("Lift power: ", Range.clip(gamepad2.right_stick_y, -1, 1));
            //telemetry.addData("Intake Position: ", intakePower);
            //telemetry.addData("lift target: ", liftTarget);
            telemetry.update();
        }


    }

    public void moveArm(double power, double distance) {
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
