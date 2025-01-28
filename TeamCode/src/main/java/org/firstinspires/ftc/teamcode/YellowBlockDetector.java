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
    final double KNOWN_WIDTH = 8.0; // Real-world width of the block in centimeters
    final double FOCAL_LENGTH = 940; //554; //1430.0; // Focal length in pixels (calibrate this value)

    public YellowBlockDetector(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat hsvMat = new Mat();
        Mat yellowMask = new Mat();
        Mat hierarchy = new Mat();

        // Convert the image to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Define the yellow color range in HSV
        Scalar lowerYellow = new Scalar(20, 100, 100); // Adjust based on lighting
        Scalar upperYellow = new Scalar(30, 255, 255);

        // Create a mask for yellow color
        Core.inRange(hsvMat, lowerYellow, upperYellow, yellowMask);

        // Morphological operations to clean up the mask
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours of the yellow objects
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Iterate through contours to detect blocks and their orientation
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 1000) { // Filter small noise
                // Approximate the contour to a polygon
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxCurve, 0.02 * Imgproc.arcLength(contour2f, true), true);

                // Draw the contour
                Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);

                // Calculate the bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(new MatOfPoint(approxCurve.toArray()));
                double pixelWidth = boundingRect.width;
                // Calculate distance
                double distance = (KNOWN_WIDTH * FOCAL_LENGTH) / pixelWidth;
                Moments moments = Imgproc.moments(contour);
                double cx = moments.get_m10() / moments.get_m00(); // X center
                double cy = moments.get_m01() / moments.get_m00(); // Y center

                // Orientation
                double angle = Imgproc.fitEllipse(contour2f).angle;
                // Draw the rectangle
                Imgproc.rectangle(input, boundingRect, new Scalar(255, 0, 0), 2);
                //Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(0, 0, 255), -1);

                // Display telemetry data
                telemetry.addData("Block Center", "X: %.2f, Y: %.2f", cx, cy);
                telemetry.addData("Orientation Angle, DIST(IN)", "%.2f, %.2f", angle, distance / 2.54);
            }
        }

        // Release resources
        hsvMat.release();
        yellowMask.release();
        hierarchy.release();

        return input;
    }
}
