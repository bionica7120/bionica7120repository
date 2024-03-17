package org.firstinspires.ftc.teamcode.testbot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class TestBotHardwareMap {

    //used to instantiate objects
    HardwareMap hwMap;

    //creation and instantiation of objects
    public DcMotor leftFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    //public DcMotor linearLift;
    //public Servo intake;
    public IMU imu;

    public void init(HardwareMap hwMap) {

        //leftFront
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setPower(0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftBack
        leftBackDrive = hwMap.get(DcMotor.class, "leftBackDrive");
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setPower(0);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightFront
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setPower(0);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //rightBack
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setPower(0);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //linearLift
        //linearLift = hwMap.get(DcMotor.class, "linearLift");
        //linearLift.setDirection(DcMotor.Direction.FORWARD);
        //linearLift.setPower(0);
        //linearLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //linearLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        //intake = hwMap.get(Servo.class, "intake");

        //field centric stuff
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);



    }

}
