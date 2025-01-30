package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class YellowBlockDetector extends OpenCvPipeline {
    private Telemetry telemetry;

    // Known parameters (calibrate these for your setup)
    final double FOCAL_LENGTH = 920; // Focal length in pixels (calibrate this value)

    public YellowBlockDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        Mat yellowMask = new Mat();
        Mat morphedMask = new Mat();
        Mat hierarchy = new Mat();
        Rect targetBlock = null;
        double bestOrientationAngle = 0.0;

        // Known parameters
        final double KNOWN_WIDTH = 8.0; // Real-world width of the block in cm
        final double FOCAL_LENGTH = 960; // Calibrated focal length in pixels
        final int noPickZoneLeftThreshold = 300; // Pixels from the left edge to ignore
        final int noPickZoneBottomThreshold = 50; // Pixels from the bottom edge to ignore
        final int noPickZoneTopThreshold = 50; // Pixels from the top edge to ignore

        try {
            // Convert the image to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Define yellow range in HSV
            Scalar lowerYellow = new Scalar(20, 100, 100); // Adjust for lighting
            Scalar upperYellow = new Scalar(30, 255, 255);
            Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);

            // Apply morphological operations
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.erode(yellowMask, yellowMask, kernel);
            Imgproc.dilate(yellowMask, yellowMask, kernel);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Center of the frame
            int frameCenterX = input.width() / 2;
            int frameCenterY = input.height() / 2;
            double closestToFrameCenter = Double.MAX_VALUE; // Smallest distance to frame center

            // Process each contour
            for (MatOfPoint contour : contours) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                double area = Imgproc.contourArea(contour);

                // Skip blocks in the no-pick zones
                if (boundingRect.x < noPickZoneLeftThreshold ||
                        boundingRect.y + boundingRect.height > input.height() - noPickZoneBottomThreshold ||
                        boundingRect.y < noPickZoneTopThreshold) {
                    Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 255), 2); // Yellow for ignored blocks
                    continue;
                }

                // Filter out small noise or invalid shapes
                if (area > 1000) {
                    Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 2); // Green for valid blocks

                    // Calculate the block's center
                    int blockCenterX = boundingRect.x + (boundingRect.width / 2);
                    int blockCenterY = boundingRect.y + (boundingRect.height / 2);

                    // Calculate distance to the frame center
                    double distanceToFrameCenter = Math.sqrt(
                            Math.pow(blockCenterX - frameCenterX, 2) + Math.pow(blockCenterY - frameCenterY, 2)
                    );

                    // Update target block if closer to the center of the frame
                    if (distanceToFrameCenter < closestToFrameCenter) {
                        closestToFrameCenter = distanceToFrameCenter;
                        targetBlock = boundingRect;

                        // Calculate the orientation angle for the target block
                        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                        RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

                        if (rotatedRect.size.width > rotatedRect.size.height) {
                            bestOrientationAngle = rotatedRect.angle;
                        } else {
                            bestOrientationAngle = rotatedRect.angle + 90;
                        }

                        if (bestOrientationAngle < 0) {
                            bestOrientationAngle += 180;
                        }
                    }
                }
            }

            // Highlight the selected target block
            if (targetBlock != null) {
                Imgproc.rectangle(input, targetBlock, new Scalar(255, 0, 0), 4); // Blue for target block

                // Calculate the distance to the target block
                double pixelWidth = targetBlock.width;
                double pixelHeight = targetBlock.height;
                double aspectRatio = (double) pixelWidth / pixelHeight;

                if (aspectRatio > 1.0) {
                    telemetry.addData("Horizontal", "PW %.2f", pixelWidth);
                } else {
                    telemetry.addData("Vertical", "PW %.2f", pixelWidth);
                    pixelWidth = pixelWidth * 2;
                }

                double distanceToTarget = ((KNOWN_WIDTH * FOCAL_LENGTH) / pixelWidth) / 2.54; // Convert to inches
                // Telemetry for the target block
                telemetry.addData("Target Block", "X: %d, Y: %d", targetBlock.x, targetBlock.y);
                telemetry.addData("Orientation Angle", "%.2f°", bestOrientationAngle);
                telemetry.addData("Distance to Target", "%.2f INCH", distanceToTarget);
            }
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        } finally {
            // Release resources
            hsvMat.release();
            yellowMask.release();
            morphedMask.release();
            hierarchy.release();
        }

        return input;
    }


}
