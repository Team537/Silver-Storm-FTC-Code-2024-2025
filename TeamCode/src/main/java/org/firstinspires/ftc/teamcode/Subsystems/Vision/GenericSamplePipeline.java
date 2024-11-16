package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Canvas;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import java.text.DecimalFormat;
import java.util.ArrayList;

public class GenericSamplePipeline extends OpenCvPipeline {

    // Settings
    private final Scalar LOW_COLOR_RANGE = new Scalar(10,175,170);
    private final Scalar HIGH_COLOR_RANGE = new Scalar(30,255,255);

    // Storage
    private Telemetry telemetry;
    private Size imageSize;
    private Mat intrinsicCameraMatrix;
    private Mat distortionCoefficients;
    private Mat translationMatrix;
    private Mat rotationMatrix;

    private double cameraDistanceFromGroundMeters = 0.0254; //TODO: Fill out field with actual value. Note: measured from robot origin to ground.

    private Mat mapX = new Mat();
    private Mat mapY = new Mat();
    private Mat newIntrinsicCameraMatrix = new Mat();
    private Rect regionOfInterest = new Rect();

    // Flags
    private boolean calculatedUndistortedImageValues = false;
    private volatile boolean shouldCaptureFrame = false;

    /**
     * Creates a new GenericSamplePipeline that will process images of the given dimensions.
     *
     * @param imageWidth The width of the processed image.
     * @param imageHeight The height of the processed image.
     */
    public GenericSamplePipeline(int imageWidth, int imageHeight, Telemetry telemetry) {
        this.imageSize = new Size(imageWidth, imageHeight);
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat inputFrame) {

        // Clone the input frame to preserve the original.
        Mat outputFrame = inputFrame.clone();

        // If a valid camera matrix and distortion coefficient matrix were provided, remove distortion from the image.
        if (intrinsicCameraMatrix != null && distortionCoefficients != null) {
            outputFrame = undistortImage(outputFrame);
        }

        // Create a mask to isolate yellow regions in the image.
        Mat yellowMask = createYellowMask(outputFrame);
        if (yellowMask == null) {
            return outputFrame; // If no mask is created, return the original input.
        }

        // Get contours and bounding boxes from the masked image.
        ArrayList<MatOfPoint> contours = findContours(yellowMask);
        Rect[] boundingBoxes = extractBoundingBoxes(contours);
        yellowMask.release();

        // If at least one bounding box is found, process and return the updated frame.
        if (boundingBoxes.length > 0) {

            // Draw bounding boxes on detected objects.
            drawBoundingBoxes(outputFrame, boundingBoxes);
        }

        // If requested, save a photo of the processed and unprocessed video frames.
        if (shouldCaptureFrame) {
            saveMatToDisk(inputFrame, "InputFrame-" + System.currentTimeMillis());
            saveMatToDisk(outputFrame, "OutputFrame-" + System.currentTimeMillis());
            shouldCaptureFrame = false;
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
            newIntrinsicCameraMatrix = calculateUndistortionValues();
            calculatedUndistortedImageValues = true; // Mark that the undistortion map values and new camera matrix have been calculated.
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
     * Calculates all values required to undistort an image.
     *
     * @return Returns the new camera matrix for the undistorted image.
     */
    private Mat calculateUndistortionValues() {

        // Initialize undistortion and rectification transformation maps (mapX and mapY) using the original
        // camera matrix and distortion coefficients. These maps are used to remap the pixels of the input frame.
        Calib3d.initUndistortRectifyMap(intrinsicCameraMatrix, distortionCoefficients, new Mat(), newIntrinsicCameraMatrix,
                imageSize, CvType.CV_32FC1, mapX, mapY);

        // Calculate and return the optimal new camera matrix, adjusting the field of view while minimizing distortion.
        // The regionOfInterest (ROI) defines the valid part of the undistorted image, eliminating any black borders
        // introduced by the remapping process. This new camera matrix will be used for cropping the image.
        return Calib3d.getOptimalNewCameraMatrix(intrinsicCameraMatrix, distortionCoefficients,
                imageSize, 1, imageSize, regionOfInterest);
    }

    /**
     * Creates a binary mask that isolates yellow regions in the input frame.
     *
     * @param inputFrame The original input image.
     * @return A binary mask where yellow areas are white, and the rest is black.
     */
    @Nullable
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
        hsvImage.release();

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
    private ArrayList<MatOfPoint> findContours(Mat mask) {

        // Detect edges in the mask using the Canny edge detector.
        Mat edges = new Mat();
        Imgproc.Canny(mask, edges, 400, 500);

        // Find contours from the detected edges.
        ArrayList<MatOfPoint> contours = new ArrayList<>();
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
    private Rect[] extractBoundingBoxes(ArrayList<MatOfPoint> contours) {

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

        // Keep track of the largest bounding box so that it's estimation can be displayed on the telemetry.
        double greatestArea = 0;
        String largestBoundingBoxRoundedXPositionMeters = "";
        String largestBoundingBoxRoundedYPositionMeters = "";
        String largestBoundingBoxRoundedZPositionMeters = "";

        // Loop through each bounding box and draw it on the output frame.
        for (Rect box : boundingBoxes) {

            // Draw a rectangle around each detected object.
            Imgproc.rectangle(outputFrame, box, new Scalar(0, 0, 0), 3);
            Imgproc.rectangle(outputFrame, box, new Scalar(135, 114, 9), 1);

            // Provide a label identifying the detected object.
            Point labelPositionL = new Point(box.x, box.y);
            Imgproc.putText(outputFrame, "Neutral Sample", labelPositionL, Imgproc.FONT_HERSHEY_DUPLEX,
                    0.8, new Scalar(0, 0, 0), 3); //  Outline
            Imgproc.putText(outputFrame, "Neutral Sample", labelPositionL, Imgproc.FONT_HERSHEY_DUPLEX,
                    0.8, new Scalar(255, 221, 51), 1); // Text

            // Attempt to estimate the 3D position of the detected object.
            if (newIntrinsicCameraMatrix != null && rotationMatrix != null && translationMatrix != null) {

                // Estimate the detected object's 3D position.
                Mat object3DPosition = estimate3DPosition(box);

                // Round all estimated positional values so that the image can be more clearly read and distinguished.
                DecimalFormat decimalFormat = new DecimalFormat("#.####");
                String roundedXPositionMeters = decimalFormat.format(object3DPosition.get(0, 0)[0]);
                String roundedYPositionMeters =  decimalFormat.format(object3DPosition.get(0, 1)[0]);
                String roundedZPositionMeters =  decimalFormat.format(object3DPosition.get(0, 2)[0]);

                // Create a message that can be displayed on the image.
                String positionText = "(" + roundedXPositionMeters + "," + roundedYPositionMeters + "," +
                        roundedZPositionMeters + ")";

                // Display the object's estimated position on screen.
                Point positionLabelPosition = new Point(box.x - (box.width / 2.0), box.y + box.height);
                Imgproc.putText(outputFrame, positionText, positionLabelPosition, Imgproc.FONT_HERSHEY_DUPLEX,
                        0.8, new Scalar(0, 0, 0), 3); // Outline
                Imgproc.putText(outputFrame, positionText, positionLabelPosition, Imgproc.FONT_HERSHEY_DUPLEX,
                        0.8, new Scalar(255, 221, 51), 1); // Text

                if (box.area() > greatestArea) {
                    greatestArea = box.area();
                    largestBoundingBoxRoundedXPositionMeters = roundedXPositionMeters;
                    largestBoundingBoxRoundedYPositionMeters = roundedYPositionMeters;
                    largestBoundingBoxRoundedZPositionMeters = roundedZPositionMeters;
                }
            }
        }

        // Display the most prominent estimated object's position.
        telemetry.addData("Estimated X", largestBoundingBoxRoundedXPositionMeters);
        telemetry.addData("Estimated Y", largestBoundingBoxRoundedYPositionMeters);
        telemetry.addData("Estimated Z", largestBoundingBoxRoundedZPositionMeters);
    }

    /**
     * Estimates the given detected object's position in 3D space, assuming that the object is on the floor.
     *
     * @param detectedObject The bounding box around the detected object, as a Rect.
     * @return A new 1x3 matrix containing an estimate of the 3D position of the detected object with
     *         values in the following order: X, Y, Z
     */
    private Mat estimate3DPosition(Rect detectedObject) {

        /*
         Since the pixel coordinates are from the top left corner of the bounding box, adjust them
         so that they are the bottom center of the bounding box.

         NOTE: The Y coordinate that's used must be from the bottom of the bounding box because we are assuming
         that the object is a set position off of the ground. The bottom of the bounding box is where
         object's geometry touches the ground,  thus we must use it.
         */
        double adjustedYPixelCoordinate = detectedObject.y + detectedObject.height;
        double adjustedXPixelCoordinate = detectedObject.x + (detectedObject.width / 2.0);

        /*
        Estimate the object's position.
        This is done using the inverse of the following equation: P (Camera) = K * (R | T) * P (World)

        Where:
         - K is the camera matrix,
         - R is a matrix representing the camera rotation relative to the world,
         - T is a matrix representing the camera's transformation relative to the world
         - P (Camera) is a matrix containing the object's pixel coordinates.
         - P (World) is a matrix containing the object's coordinates relative to the world.
         */
        double objectDepthMeters = estimateObjectDepth(adjustedYPixelCoordinate);
        double xPositionMeters = estimateObjectX(objectDepthMeters, adjustedXPixelCoordinate);

        // TODO: Convert camera centric coordinates to robot centric coordinates.

        // Represent the estimated position as a Mat object.
        Mat objectWorldCoordinates = new Mat(1,3, CvType.CV_32FC1);
        objectWorldCoordinates.put(0, 0, xPositionMeters, -cameraDistanceFromGroundMeters, objectDepthMeters);

        // Return the object's estimated 3D position as a matrix object.
        return objectWorldCoordinates; // 3x1 matrix containing [X, Y, Z]
    }

    /**
     * Estimates the object's distance from the camera based on the object's Y pixel coordinate.
     * We are assuming that the object is a set distance away from
     *
     * @param objectYPixelCoordinate The Y pixel coordinate of the detected object.
     * @return The estimated scaling factor.
     */
    private double estimateObjectDepth(double objectYPixelCoordinate) {
        double yDistanceFromCamera = cameraDistanceFromGroundMeters; // Known height of the camera above the ground
        double focalLengthY = newIntrinsicCameraMatrix.get(1, 1)[0]; // fy from the intrinsic camera matrix
        double principlePointY = newIntrinsicCameraMatrix.get(1, 2)[0]; // cy from the intrinsic camera matrix

        // Calculate and return the object's depth.
        return (focalLengthY * yDistanceFromCamera) / (objectYPixelCoordinate - principlePointY);
    }

    /**
     * Estimates the object's distance along the X axis in 3D space, relative to the camera.
     *
     * @param objectDepthMeters The detected object's distance from the camera.
     * @param objectXPixelCoordinate The X pixel coordinate of the detected object.
     * @return The estimated scaling factor.
     */
    private double estimateObjectX(double objectDepthMeters, double objectXPixelCoordinate) {
        double focalLengthX = newIntrinsicCameraMatrix.get(0, 0)[0]; // fx from the intrinsic camera matrix
        double principlePointX = newIntrinsicCameraMatrix.get(0, 2)[0]; // cx from the intrinsic camera matrix

        // Calculate and return the object's X position.
        return  (((objectXPixelCoordinate - principlePointX) * objectDepthMeters) / focalLengthX);
    }

    /**
     * Sets the extrinsic camera matrix using the provided rotation and transformation matrices.
     *
     * @param cameraRotationMatrix A Mat representing the camera's rotation relative to the robot's origin.
     * @param cameraTransformationMatrix A Mat representing the camera's position relative to the robot's origin.
     */
    public void setExtrinsicCameraMatrix(Mat cameraRotationMatrix, Mat cameraTransformationMatrix) {

        // Set this GenericSamplePipeline's rotation and translation matrices.
        this.rotationMatrix = cameraRotationMatrix;
        this.translationMatrix = cameraTransformationMatrix;
    }

    /**
     * Sets the intrinsic camera matrix for this pipeline.
     *
     * @param intrinsicCameraMatrix A 3x3 matrix representing the camera's intrinsic parameters,
     *                     including focal lengths and optical center.
     */
    public void setIntrinsicCameraMatrix(Mat intrinsicCameraMatrix) {

        // Set this GenericSamplePipeline's intrinsic camera matrix.
        this.intrinsicCameraMatrix = intrinsicCameraMatrix;

        // The values required to remove distortion must be recalculated.
        this.calculatedUndistortedImageValues = false;
    }

    /**
     * Sets the distortion coefficients for the camera.
     *
     * @param distortionCoefficients A matrix containing distortion coefficients that
     *                               correct lens distortion effects in captured images.
     */
    public void setDistortionCoefficients(Mat distortionCoefficients) {

        // Set this GenericSamplePipeline's distortion coefficient matrix.
        this.distortionCoefficients = distortionCoefficients;

        // The values required to remove distortion must be recalculated.
        this.calculatedUndistortedImageValues = false;
    }

    /**
     * Saves two images of what the camera sees to the driver hub. One of the input frame (unprocessed image),
     * and one of the processed frame (input frame wth annotations and boxes drawn over detected objects).
     */
    public void captureFrame() {
        shouldCaptureFrame = true;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}