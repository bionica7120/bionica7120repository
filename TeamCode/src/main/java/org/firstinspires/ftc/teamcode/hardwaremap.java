package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class hardwaremap {

    //used to instantiate objects
    HardwareMap hwMap;

    //creation and instantiation of objects
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotorEx arm;
    public DcMotor intake;
    public DcMotorEx suspension;
    public Servo drone;
    public Servo intakeWrist;
    public IMU imu;
    public Encoder parallelEncoder, perpendicularEncoder;

    public void init(HardwareMap hwMap) {

        //leftFront
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setPower(0);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftBack
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setPower(0);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightFront
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setPower(0);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightBack
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setPower(0);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //arm
        arm = hwMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setPower(0);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //suspension
        suspension = hwMap.get(DcMotorEx.class, "suspension");
        suspension.setDirection(DcMotorEx.Direction.FORWARD);
        suspension.setPower(0);
        suspension.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //drone
        drone = hwMap.get(Servo.class, "drone");

        //intakeWrist
        intakeWrist = hwMap.get(Servo.class, "intakeWrist");

        //imu
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //encoders
        parallelEncoder = new Encoder(hwMap.get(DcMotorEx.class, "arm"));
        perpendicularEncoder = new Encoder(hwMap.get(DcMotorEx.class, "suspension"));
    }

    public void power(double output) {
        leftBackDrive.setPower(-output);
        rightBackDrive.setPower(output);
        leftFrontDrive.setPower(-output);
        rightFrontDrive.setPower(output);

    }

    public void mecanumDrive(double x, double y, double r) {
        double frontLeftPower = y + x + r;
        double frontRightPower = y - x - r;
        double backLeftPower = y - x + r;
        double backRightPower = y + x - r;

        rightBackDrive.setPower(backLeftPower);
        leftBackDrive.setPower(backRightPower);
        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
    }

}
