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
import com.sun.tools.javac.util.List;

public class GenericSamplePipeline extends OpenCvPipeline {

    // Settings
    private final Scalar LOW_COLOR_RANGE = new Scalar(10,140,135);
    private final Scalar HIGH_COLOR_RANGE = new Scalar(50,255,255);

    // Storage
    private Size imageSize;
    private Mat intrinsicCameraMatrix;
    private Mat distortionCoefficients;
    private Mat extrinsicCameraMatrix; // A matrix that contains the camera's rotation and translation from the robot's origin
    private Mat invertedCameraProjectionMatrix;

    private double cameraDistanceFromGroundMeters = 1; //TODO: Fill out field with actual value. Note: measured from robot origin to ground.

    private Mat mapX, mapY;
    private Mat newIntrinsicCameraMatrix;
    private Rect regionOfInterest = new Rect();

    // Flags
    private boolean calculatedUndistortedImageValues = false;
    private boolean calculatedInvertedCameraProjectionMatrix = false;

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

        // Loop through each bounding box and draw it on the output frame.
        for (Rect box : boundingBoxes) {

            // Draw a rectangle around each detected object.
            Imgproc.rectangle(outputFrame, box, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(outputFrame, box, new Scalar(0, 0, 0), 1);

            // Place a label above the detected object.
            Point labelPosition = new Point(box.x, box.y);
            Imgproc.putText(outputFrame, "Detected Object", labelPosition, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(196, 85, 8));

            // Attempt to estimate the 3D position of the detected object.
            if (newIntrinsicCameraMatrix != null && extrinsicCameraMatrix != null) {
                Mat object3DPosition = estimate3DPosition(box);
                Imgproc.putText(outputFrame, "(" + object3DPosition.get(0, 0)[0] + "," +
                        object3DPosition.get(1, 0)[0] + "," + object3DPosition.get(2, 0)[0] +
                        ")", labelPosition, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(196, 85, 8));
            }
        }
    }

    /**
     * Estimates the given detected object's position in 3D space, assuming that the object is on the floor.
     *
     * @param detectedObject The bounding box around the detected object, as a Rect.
     * @return A new 3x1 matrix containing an estimate of the 3D position of the detected object with
     *         values in the following order: X, Y, Z
     */
    private Mat estimate3DPosition(Rect detectedObject) {

        // Store the pixel coordinates of the detected object in an array.
        double[] objectPixelCoordinateValues = new double[] {
                detectedObject.x,
                detectedObject.y + detectedObject.height, // Y coordinate from the bottom of the bounding box
                1
        };

        // Store the pixel coordinates of the detected object in a new Mat object.
        Mat objectPixelCoordinates = new Mat(3, 1, CvType.CV_64FC1);
        objectPixelCoordinates.put(0, 0, objectPixelCoordinateValues);

        // Estimate the scaling factor (Z value or depth)
        double scalingFactor = estimateScalingFactor(objectPixelCoordinates.get(1, 0)[0]);

        // Scale the pixel coordinates (u, v, 1) by the scaling factor (Z value)
        Mat scaledObjectPixelCoordinates = new Mat();
        Core.multiply(objectPixelCoordinates, new Scalar(scalingFactor), scaledObjectPixelCoordinates);

        // If the inverted camera projection matrix hasn't been calculated, then calculate it.
        if (!calculatedInvertedCameraProjectionMatrix) {
            invertedCameraProjectionMatrix = calculateInvertedCameraProjectionMatrix();
            calculatedInvertedCameraProjectionMatrix = true; // The inverted camera projection matrix has been calculated.
        }

        // Now we transform the scaled 2D pixel coordinates into 3D world coordinates
        Mat objectWorldCoordinates = new Mat();
        Core.gemm(invertedCameraProjectionMatrix, scaledObjectPixelCoordinates, 1, new Mat(), 0, objectWorldCoordinates);

        // Return the object's estimated 3D position, as a new matrix.
        return objectWorldCoordinates; // 3x1 matrix containing [X, Y, Z]
    }

    /**
     * Estimates the scaling factor (Z value or depth) based on the object's Y pixel coordinate.
     *
     * @param objectYPixelCoordinate The Y pixel coordinate of the detected object.
     * @return The estimated scaling factor.
     */
    private double estimateScalingFactor(double objectYPixelCoordinate) {
        double yDistanceFromCamera = cameraDistanceFromGroundMeters; // Known height of the camera above the ground
        double focalLengthY = newIntrinsicCameraMatrix.get(1, 1)[0]; // fy from the intrinsic camera matrix
        double principlePointY = newIntrinsicCameraMatrix.get(1, 2)[0]; // cy from the intrinsic camera matrix

        return (focalLengthY * yDistanceFromCamera) / (objectYPixelCoordinate - principlePointY);
    }

    /**
     * Calculates the inverted camera projection matrix by combining intrinsic and extrinsic matrices.
     *
     * @return The calculated inverted camera projection matrix.
     */
    private Mat calculateInvertedCameraProjectionMatrix() {

        // Calculate the combined projection matrix from the intrinsic and extrinsic matrices
        Mat cameraProjectionMatrix = new Mat();
        Core.gemm(extrinsicCameraMatrix, newIntrinsicCameraMatrix, 1, new Mat(), 0, cameraProjectionMatrix);

        // Invert the camera's projection matrix
        Mat invertedMatrix = new Mat();
        Core.invert(cameraProjectionMatrix, invertedMatrix);

        // Return the inverted camera projection matrix
        return invertedMatrix;
    }

    /**
     *
     * @param cameraRotationMatrix The camera's rotation relative to the robot's origin, as a Mat.
     * @param cameraTransformationMatrix The camera's position relative to the robot's origin, as a Mat.
     */
    public void setExtrinsicCameraMatrix(Mat cameraRotationMatrix, Mat cameraTransformationMatrix) {

        // Initialize an empty matrix to store both the cameraTransformationMatrix and cameraRotationMatrix
        this.extrinsicCameraMatrix = new Mat(3, 4, CvType.CV_64FC1);

        /*
         * Combine the cameraTransformationMatrix and cameraRotationMatrix matrices such that the result
         * contains the values of the rotation matrix followed by the values of the transformation matrix.
         * Then store the resulting matrix in the newly initialized extrinsicCameraMatrix matrix.
         * The result will look something like the following: [r1, r2, r3, tx].
         */
        Core.hconcat(List.of(cameraRotationMatrix, cameraTransformationMatrix), extrinsicCameraMatrix);

        // The inverse of the camera projection matrix must be recalculated.
        this.calculatedInvertedCameraProjectionMatrix = false;
    }

    /**
     * Sets the intrinsic camera matrix for this pipeline.
     *
     * @param intrinsicCameraMatrix A 3x3 matrix representing the camera's intrinsic parameters,
     *                     including focal lengths and optical center.
     */
    public void setIntrinsicCameraMatrix(Mat intrinsicCameraMatrix) {

        // Set the intrinsic camera matrix.
        this.intrinsicCameraMatrix = intrinsicCameraMatrix;

        // The values required to remove distortion must be recalculated.
        this.calculatedUndistortedImageValues = false;

        // The inverse of the camera projection matrix must be recalculated.
        this.calculatedInvertedCameraProjectionMatrix = false;
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
        this.calculatedUndistortedImageValues = false;

        // The inverse of the camera projection matrix must be recalculated. While the distortion
        // coefficients may not be directly used to estimate the 3D position of an object, they
        // influence the newCameraMatrix, which is used during the position estimation process.
        this.calculatedInvertedCameraProjectionMatrix = false;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}