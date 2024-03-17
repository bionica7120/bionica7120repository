package org.firstinspires.ftc.teamcode.regionalsAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "TrajectoryAuto")

public class TrajectoryAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int condition = 3;

        Trajectory center1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(28)
                .build();

        Trajectory center2 = drive.trajectoryBuilder(center1.end())
                .back(5)
                .build();

        Trajectory center3 = drive.trajectoryBuilder(center2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .back(28)
                .build();

        Trajectory left1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(28)
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
                .strafeLeft(5)
                .build();

        Trajectory left3 = drive.trajectoryBuilder(left2.end())
                .forward(38)
                .build();

        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .strafeRight(5)
                .build();

        Trajectory right1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(28)
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end().plus(new Pose2d(0, 0, Math.toRadians(-90))), false)
                .back(28)
                .build();




        waitForStart();

        if(isStopRequested()) return;


        if (condition == 1)
        {
            drive.followTrajectory(center1);
            sleep(500);
            drive.intakeOpenOrClose(1, 3000);
            sleep(500);
            drive.followTrajectory(center2);
            sleep(500);
            drive.turn(Math.toRadians(-90));
            sleep(500);
            drive.followTrajectory(center3);
            sleep(500);
            drive.moveArm(0.5, 4000);
            sleep(500);
            drive.intakeOpenOrClose(1, 3000);
            sleep(500);
            drive.moveArm(-0.5, 1000);
            drive.moveArm(0.2, 250);

            return;

        }

        if (condition == 2)
        {
            drive.followTrajectory(left1);
            sleep(500);
            drive.turn(Math.toRadians(90));
            sleep(500);
            drive.intakeOpenOrClose(1, 3000);
            sleep(500);
            drive.followTrajectory(left2);
            sleep(500);
            drive.followTrajectory(left3);
            sleep(500);
            drive.followTrajectory(left4);
            sleep(500);
            drive.turn(Math.toRadians(180));
            sleep(500);
            drive.moveArm(0.5, 4000);
            sleep(500);
            drive.intakeOpenOrClose(1, 3000);
            sleep(500);
            drive.moveArm(-0.5, 1000);
            drive.moveArm(0.2, 250);

        }

        if (condition == 3)
        {
            drive.followTrajectory(right1);
            sleep(500);
            drive.turn(Math.toRadians(-90));
            sleep(500);
            drive.intakeOpenOrClose(1, 3000);
            sleep(500);
            drive.followTrajectory(right2);
            sleep(500);
            drive.moveArm(0.5, 4000);
            sleep(500);
            drive.intakeOpenOrClose(1, 3000);
            sleep(500);
            drive.moveArm(-0.5, 1000);
            drive.moveArm(0.2, 250);


        }



    }
}
