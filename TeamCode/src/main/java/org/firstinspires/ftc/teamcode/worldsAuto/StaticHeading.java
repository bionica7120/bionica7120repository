package org.firstinspires.ftc.teamcode.worldsAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwaremap;

/**
 * Robot's heading adjuster with PID control
 */
@TeleOp(name = "Static Heading")
public class StaticHeading extends LinearOpMode {
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    hardwaremap drivetrain = new hardwaremap();

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // Allows to use FTCDASH
        drivetrain.init(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; // Radian Angle measure
        imu.initialize(parameters);

        double referenceAngle = Math.toRadians(90); // reference angle -- input is in degrees
        waitForStart();

//        while (!isStarted()) {
//            drivetrain.imu.resetYaw();
//            telemetry.addData("Target IMU Angle", referenceAngle);
//            telemetry.addData("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//            telemetry.update();
//        }

        while(opModeIsActive()){
            telemetry.addData("Target IMU Angle in Degrees: ", Math.toDegrees(referenceAngle));
           // telemetry.addData("Current IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Current IMU Reading", drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            double power = PIDControl(referenceAngle,  drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            drivetrain.power(power);
            telemetry.update();
        }

    }
    // Reference -- target angle        state -- current Angle
    public double PIDControl(double reference, double state)
    {
        double error = angleWrap(reference - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    // makes the robot take the most efficient route to achieve reference angle.
    // For example, robot's at 0 degrees, and reference is 90. this method make sure that the robot
    // travels pi/2 radians instead of 3pi/2 radians
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


}
