package org.firstinspires.ftc.teamcode.worldsAuto;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous
public class AprilTagsAuto extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    String telemetrystatement = "";
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.27;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    // Tag ID 18 from the 36h11 family
    int LEFT_BLUE = 1;  //according to april tags
    int MIDDLE_BLUE = 2;
    int RIGHT_BLUE = 3;

    int LEFT_RED = 4;
    int MIDDLE_RED = 5;
    int RIGHT_RED = 6;



    AprilTagDetection tagOfInterest = new AprilTagDetection();

    @Override
    public void runOpMode() {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT_BLUE || tag.id == MIDDLE_BLUE || tag.id == RIGHT_BLUE || tag.id == LEFT_RED || tag.id == MIDDLE_RED || tag.id == RIGHT_RED) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

            }
            telemetry.addLine(telemetrystatement);
            telemetry.update();
            sleep(20);
        }



        while (opModeIsActive()) {


        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(telemetrystatement);
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    class VisionPortalStreamingOpMode extends LinearOpMode {
         class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
            private final AtomicReference<Bitmap> lastFrame =
                    new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

            @Override
            public void init(int width, int height, CameraCalibration calibration) {
                lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
            }

            @Override
            public Object processFrame(Mat frame, long captureTimeNanos) {
                Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(frame, b);
                lastFrame.set(b);
                return null;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                    float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                    Object userContext) {
                // do nothing
            }

            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
                continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
            }
        }

        @Override
        public void runOpMode() throws InterruptedException {
//            final org.firstinspires.ftc.teamcode.qualifiers.VisionPortalStreamingOpMode.CameraStreamProcessor processor = new org.firstinspires.ftc.teamcode.qualifiers.VisionPortalStreamingOpMode.CameraStreamProcessor();
//
//            new VisionPortal.Builder()
//                    .addProcessor(processor)
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .build();
//
//            FtcDashboard.getInstance().startCameraStream(processor, 0);

            waitForStart();

            while (opModeIsActive()) {
                sleep(100L);
            }



        }
    }

}
