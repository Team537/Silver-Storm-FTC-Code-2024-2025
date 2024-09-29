package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Canvas;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GenericSamplePipeline extends OpenCvPipeline {

    private Scalar LOW_COLOR_RANGE = new Scalar(10,140,125);  //(10,200,165); //(15,199,200)
    private Scalar HIGH_COLOR_RANGE = new Scalar(30,255,255); //(30,255,255); //(28,255,255)

    @Override
    public Mat processFrame(Mat inputFrame) {

        // Create a mask to isolate yellow regions in the image.
        Mat yellowMask = createYellowMask(inputFrame);
        if (yellowMask == null) {
            return inputFrame; // If no mask is created, return the original input.
        }

        // Get contours and bounding boxes from the masked image.
        List<MatOfPoint> contours = findContours(yellowMask);
        Rect[] boundingBoxes = extractBoundingBoxes(contours);

        // If at least one bounding box is found, process and return the updated frame.
        if (boundingBoxes.length > 0) {

            // Clone the input frame to preserve the original.
            Mat outputFrame = inputFrame.clone();

            // Draw bounding boxes on detected objects.
            drawBoundingBoxes(outputFrame, boundingBoxes);

            // Return the frame with bounding boxes.
            return outputFrame;
        } else {
            return inputFrame; // No objects detected, return the original input.
        }
    }

    /**
     * Creates a binary mask that isolates yellow regions in the input frame.
     *
     * @param inputFrame The original input image.
     * @return A binary mask where yellow areas are white, and the rest is black.
     */
    private Mat createYellowMask(Mat inputFrame) {

        // Convert the input frame from RGB to HSV color space.
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(inputFrame, hsvImage, Imgproc.COLOR_RGB2HSV);

        // If conversion failed, return null indicating no mask could be created.
        if (hsvImage.empty()) {
            return null;
        }

        // Create a threshold image that isolates yellow regions.
        Mat thresholdImage = new Mat();
        Core.inRange(hsvImage, LOW_COLOR_RANGE, HIGH_COLOR_RANGE, thresholdImage);

        // Apply morphological operations to clean up noise and fill gaps.
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(thresholdImage, thresholdImage, Imgproc.MORPH_OPEN, kernel); // Remove noise.
        Imgproc.morphologyEx(thresholdImage, thresholdImage, Imgproc.MORPH_CLOSE, kernel); // Fill gaps.

        return thresholdImage; // Return the processed mask image.
    }

    /**
     * Finds contours in the binary mask image.
     *
     * @param mask The mask where contours will be detected.
     * @return A list of contours found in the mask.
     */
    private List<MatOfPoint> findContours(Mat mask) {

        // Detect edges in the mask using the Canny edge detector.
        Mat edges = new Mat();
        Imgproc.Canny(mask, edges, 350, 500);

        // Find contours from the detected edges.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours; // Return the list of detected contours.
    }

    /**
     * Approximates and extracts bounding rectangles around the detected contours.
     *
     * @param contours A list of contours representing detected objects.
     * @return An array of bounding rectangles around each contour.
     */
    private Rect[] extractBoundingBoxes(List<MatOfPoint> contours) {

        MatOfPoint2f[] approximatedContours = new MatOfPoint2f[contours.size()];
        Rect[] boundingBoxes = new Rect[contours.size()];

        // Approximate polygons for each contour and calculate their bounding rectangles.
        for (int i = 0; i < contours.size(); i++) {
            approximatedContours[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approximatedContours[i], 3, true);
            boundingBoxes[i] = Imgproc.boundingRect(new MatOfPoint(approximatedContours[i].toArray()));
        }

        return boundingBoxes; // Return the array of bounding rectangles.
    }

    /**
     * Draws bounding rectangles around detected objects in the input frame.
     *
     * @param outputFrame The image on which bounding boxes will be drawn.
     * @param boundingBoxes An array of bounding rectangles to be drawn.
     */
    private void drawBoundingBoxes(Mat outputFrame, Rect[] boundingBoxes) {

        // Loop through each bounding box and draw it on the output frame.
        for (Rect box : boundingBoxes) {

            // Draw a rectangle around each detected object.
            Imgproc.rectangle(outputFrame, box, new Scalar(196, 85, 8), 3);

            // Place a label above the detected object.
            Point labelPosition = new Point(box.x, box.y);
            Imgproc.putText(outputFrame, "Detected Object", labelPosition, Imgproc.FONT_HERSHEY_SIMPLEX, 1.5, new Scalar(196, 85, 8));
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
