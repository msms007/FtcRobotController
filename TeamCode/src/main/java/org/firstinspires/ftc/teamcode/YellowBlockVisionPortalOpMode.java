package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "YellowBlockVisionPortal", group = "Vision")
public class YellowBlockVisionPortalOpMode extends LinearOpMode {
    private OpenCvCamera webcam;
    private YellowBlockDetector detector;
    @Override
    public void runOpMode() {
        // Initialize the webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the detector
        detector = new YellowBlockDetector(telemetry);

        // Set the pipeline
        webcam.setPipeline(detector);

        // Open the camera and start streaming
        //webcam.openCameraDeviceAsync(() -> webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });
        telemetry.addLine("Ready to start");
        telemetry.update();

       // waitForStart();

        //while (opModeIsActive())
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addLine("Detecting yellow blocks...");
            telemetry.update();

            sleep(50); // Allow the pipeline to process frames without overwhelming the CPU
        }

        // Stop the camera stream
        webcam.stopStreaming();
    }
}

