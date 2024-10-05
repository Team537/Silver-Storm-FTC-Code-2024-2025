package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Canvas;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
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

    // Settings
    private final Scalar LOW_COLOR_RANGE = new Scalar(10,140,135);
    private final Scalar HIGH_COLOR_RANGE = new Scalar(50,255,255);

    // Storage
    private Size imageSize;
    private Mat cameraMatrix;
    private Mat distortionCoefficients;
    private Mat mapX, mapY;
    private Mat newCameraMatrix;
    private Rect regionOfInterest = new Rect();

    // Flags
    private boolean calculatedUndistortedImageValues = false;

    /**
     * Creates a new GenericSamplePipeline that will process images of the given dimensions.
     *
     * @param imageWidth The width of the processed image.
     * @param imageHeight The height of the processed image.
     */
    public GenericSamplePipeline(int imageWidth, int imageHeight) {
        this.imageSize = new Size(imageWidth, imageHeight);
    }

    @Override
    public Mat processFrame(Mat inputFrame) {

        // Clone the input frame to preserve the original.
        Mat outputFrame = inputFrame.clone();

        // If a valid camera matrix and distortion coefficient matrix were provided, remove distortion from the image.
        if (cameraMatrix != null && distortionCoefficients != null) {
            outputFrame = undistortImage(outputFrame);
        }

        // Create a mask to isolate yellow regions in the image.
        Mat yellowMask = createYellowMask(outputFrame);
        if (yellowMask == null) {
            return outputFrame; // If no mask is created, return the original input.
        }

        // Get contours and bounding boxes from the masked image.
        List<MatOfPoint> contours = findContours(yellowMask);
        Rect[] boundingBoxes = extractBoundingBoxes(contours);

        // If at least one bounding box is found, process and return the updated frame.
        if (boundingBoxes.length > 0) {

            // Draw bounding boxes on detected objects.
            drawBoundingBoxes(outputFrame, boundingBoxes);
        }

        // Return the processed image.
        return outputFrame;
    }

    /**
     * Undistorts the input image frame using pre-calculated distortion and rectification maps.
     * If the undistortion maps have not been calculated, they will be generated using the camera matrix
     * and distortion coefficients, and then the image will be cropped to remove black borders.
     *
     * @param inputFrame The original distorted image frame that needs to be undistorted.
     * @return A new Mat object representing the undistorted and cropped image.
     */
    private Mat undistortImage(Mat inputFrame) {

        // Check if the undistortion maps and optimal camera matrix have already been computed.
        if (!calculatedUndistortedImageValues) {

            // Initialize undistortion and rectification transformation maps (mapX and mapY) using the original
            // camera matrix and distortion coefficients. These maps are used to remap the pixels of the input frame.
            Calib3d.initUndistortRectifyMap(cameraMatrix, distortionCoefficients, new Mat(), newCameraMatrix,
                    imageSize, CvType.CV_32FC1, mapX, mapY);

            // Calculate the optimal new camera matrix, adjusting the field of view while minimizing distortion.
            // The regionOfInterest (ROI) defines the valid part of the undistorted image, eliminating any black borders
            // introduced by the remapping process. This new camera matrix will be used for cropping the image.
            newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients,
                    imageSize, 1, imageSize, regionOfInterest);

            // Mark that the undistortion map values and new camera matrix have been calculated.
            calculatedUndistortedImageValues = true;
        }

        // Apply the remapping to the input frame using the precomputed mapX and mapY. This undistorts the image
        // by shifting pixels to their corrected positions based on the camera matrix and distortion coefficients.
        Mat remappedImage = new Mat();
        Imgproc.remap(inputFrame, remappedImage, mapX, mapY, Imgproc.INTER_LINEAR);

        // Crop the undistorted image to the valid region of interest (ROI), removing any black borders
        // around the edges, and return the final undistorted, cropped image.
        return new Mat(remappedImage, regionOfInterest);
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
        Imgproc.morphologyEx(thresholdImage, thresholdImage, Imgproc.MORPH_OPEN, kernel); // Remove noise.

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
        Imgproc.Canny(mask, edges, 400, 500);

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
            Imgproc.rectangle(outputFrame, box, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(outputFrame, box, new Scalar(0, 0, 0), 1);

            // Place a label above the detected object.
            Point labelPosition = new Point(box.x, box.y);
            Imgproc.putText(outputFrame, "Detected Object", labelPosition, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(196, 85, 8));
        }
    }

    /**
     * Sets the camera matrix used for image calibration.
     *
     * @param cameraMatrix A 3x3 matrix representing the camera's intrinsic parameters,
     *                     including focal lengths and optical center.
     */
    public void setCameraMatrix(Mat cameraMatrix) {

        // The values required to remove distortion must be recalculated.
        this.cameraMatrix = cameraMatrix;

        // The values required to remove distortion must be recalculated.
        calculatedUndistortedImageValues = false;
    }

    /**
     * Sets the distortion coefficients for the camera.
     *
     * @param distortionCoefficients A matrix containing distortion coefficients that
     *                               correct lens distortion effects in captured images.
     */
    public void setDistortionCoefficients(Mat distortionCoefficients) {
        this.distortionCoefficients = distortionCoefficients;

        // The values required to remove distortion must be recalculated.
        calculatedUndistortedImageValues = false;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
