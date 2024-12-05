package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Vector;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;
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
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {

    // Settings
    private double cameraDistanceFromGroundMeters = 0.1095375;
    private double cameraZDistanceFromOriginMeters = 0.0508;
    private double cameraXDistanceFromOriginMeters = 0.2159;

    private double sameObjectDistanceToleranceMeters = 0.01524; // TODO: FINE TUNE

    // Storage
    private Telemetry telemetry;
    private ElapsedTime visionRuntime;
    private Function<Pose2d, Pose2d> robotToFieldSpaceConversion;

    // Sample Detection Data Storage
    private SampleType currentSampleType = SampleType.NEUTRAL;
    private Scalar lowColorRange = new Scalar(10,175,170);
    private Scalar highColorRange = new Scalar(30,255,255);

    private volatile List<Sample> detectedObjects = new ArrayList<>(); // Volatile to prevent weird scenarios with vision data updates.

    // Pipeline Settings Storage
    private Size imageSize;

    // General Processing Storage
    private Mat outputFrame = new Mat();

    // Distortion Removal Storage
    private Mat mapX = new Mat();
    private Mat mapY = new Mat();
    private Mat newIntrinsicCameraMatrix = new Mat();
    private Rect regionOfInterest = new Rect();
    private Mat remappedImage = new Mat();

    // Masking Matrix Storage
    private Mat yellowMask;
    private Mat hsvImage = new Mat();
    private Mat thresholdImage = new Mat();
    private Mat kernel;

    // Find Contours Storage
    private Mat edges = new Mat();
    private Mat hierarchy = new Mat();

    // Object 3D Position Estimation Storage
    private Mat intrinsicCameraMatrix = new Mat();
    private Mat distortionCoefficients = new Mat();
    private Mat translationMatrix = new Mat();
    private Mat rotationMatrix = new Mat();
    private Mat objectWorldCoordinates = new Mat();

    // Flags
    private boolean calculatedUndistortedImageValues = false;
    private volatile boolean shouldCaptureFrame = false;

    /**
     * Creates a new GenericSamplePipeline that will process images of the given dimensions.
     *
     * @param imageWidth The width of the processed image.
     * @param imageHeight The height of the processed image.
     * @param telemetry The robot's telemetry, for data visualization.
     * @param robotToFieldSpaceConversion A method to convert vision data to robot data.
     */
    public SampleDetectionPipeline(int imageWidth, int imageHeight, Telemetry telemetry, Function<Pose2d, Pose2d> robotToFieldSpaceConversion) {
        this.imageSize = new Size(imageWidth, imageHeight);
        this.telemetry = telemetry;
        this.visionRuntime = new ElapsedTime();
        this.robotToFieldSpaceConversion = robotToFieldSpaceConversion;
    }

    @Override
    public Mat processFrame(Mat inputFrame) {

        // Clone the input frame to preserve the original.
        this.outputFrame = inputFrame.clone();

        // If a valid camera matrix and distortion coefficient matrix were provided, remove distortion from the image.
        if (intrinsicCameraMatrix != null && distortionCoefficients != null) {
            undistortImage(outputFrame);
        }

        // Create a mask to isolate yellow regions in the image.
        createYellowMask(this.outputFrame);
        if (this.yellowMask == null) {
            return this.outputFrame; // If no mask is created, return the original input.
        }

        // Get contours and bounding boxes from the masked image.
        ArrayList<MatOfPoint> contours = findContours();
        Rect[] boundingBoxes = extractBoundingBoxes(contours);

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
     */
    private void undistortImage(Mat inputFrame) {

        // Check if the undistortion maps and optimal camera matrix have already been computed.
        if (!calculatedUndistortedImageValues) {
            newIntrinsicCameraMatrix = calculateUndistortionValues();
            calculatedUndistortedImageValues = true; // Mark that the undistortion map values and new camera matrix have been calculated.
        }

        // Apply the remapping to the input frame using the precomputed mapX and mapY. This undistorts the image
        // by shifting pixels to their corrected positions based on the camera matrix and distortion coefficients.
        Imgproc.remap(inputFrame, this.remappedImage, this.mapX, this.mapY, Imgproc.INTER_LINEAR);

        // Crop the undistorted image to the valid region of interest (ROI), removing any black borders
        // around the edges.
        this.outputFrame = new Mat(remappedImage, regionOfInterest);
    }

    /**
     * Calculates all values required to undistort an image.
     *
     * @return Returns the new camera matrix for the undistorted image.
     */
    private Mat calculateUndistortionValues() {

        // Initialize undistortion and rectification transformation maps (mapX and mapY) using the original
        // camera matrix and distortion coefficients. These maps are used to remap the pixels of the input frame.
        Calib3d.initUndistortRectifyMap(this.intrinsicCameraMatrix, this.distortionCoefficients, new Mat(), this.newIntrinsicCameraMatrix,
                this.imageSize, CvType.CV_32FC1, this.mapX, this.mapY);

        // Calculate and return the optimal new camera matrix, adjusting the field of view while minimizing distortion.
        // The regionOfInterest (ROI) defines the valid part of the undistorted image, eliminating any black borders
        // introduced by the remapping process. This new camera matrix will be used for cropping the image.
        return Calib3d.getOptimalNewCameraMatrix(this.intrinsicCameraMatrix, this.distortionCoefficients,
                this.imageSize, 1, this.imageSize, this.regionOfInterest);
    }

    /**
     * Creates a binary mask that isolates yellow regions in the input frame and saves the result to the YellowMask variable.
     *
     * @param inputFrame The original input image.
     */
    private void createYellowMask(Mat inputFrame) {

        // Clear the old yellow mask.
        this.yellowMask = null;

        // Convert the input frame from RGB to HSV color space.
        Imgproc.cvtColor(inputFrame, this.hsvImage, Imgproc.COLOR_RGB2HSV);

        // If conversion failed, return null indicating no mask could be created.
        if (this.hsvImage.empty()) {
            return;
        }

        // Create a threshold image that isolates yellow regions.
        Core.inRange(this.hsvImage, this.lowColorRange, this.highColorRange, this.thresholdImage);

        // Apply morphological operations to clean up noise and fill gaps.
        this.kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(this.thresholdImage, this.thresholdImage, Imgproc.MORPH_OPEN, kernel); // Remove noise.
        Imgproc.morphologyEx(this.thresholdImage, this.thresholdImage, Imgproc.MORPH_CLOSE, kernel); // Fill gaps.
        Imgproc.morphologyEx(this.thresholdImage, this.thresholdImage, Imgproc.MORPH_OPEN, kernel); // Remove noise.

        // Update the yellow mask.
        this.yellowMask = thresholdImage.clone();
    }

    /**
     * Finds contours in the binary mask image.
     *
     * @return A list of contours found in the mask.
     */
    private ArrayList<MatOfPoint> findContours() {

        // Detect edges in the mask using the Canny edge detector.
        Imgproc.Canny(this.yellowMask, this.edges, 400, 500);

        // Find contours from the detected edges.
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(this.edges, contours, this.hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

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
               estimate3DPosition(box);

                // Get object's position along each axis.
                double xPosition = -this.objectWorldCoordinates.get(0, 0)[0] - cameraZDistanceFromOriginMeters;
                double yPosition = this.objectWorldCoordinates.get(0, 1)[0];
                double zPosition = this.objectWorldCoordinates.get(0, 2)[0] + cameraXDistanceFromOriginMeters;

                // Create a Pose2d of the object and convert it to robot space.
                Pose2d objectPoseRobotSpace = new Pose2d(zPosition, xPosition);
                Pose2d objectPoseWorldSpace = robotToFieldSpaceConversion.apply(objectPoseRobotSpace);

                // Create a new sample object and add it to the list of detected objects.
                Sample detectedObject = new Sample(objectPoseWorldSpace, this.currentSampleType, visionRuntime.getElapsedTime(TimeUnit.SECOND));
                addToDetectedSamples(detectedObject);

                // Round all estimated positional values so that the image can be more clearly read and distinguished.
                DecimalFormat decimalFormat = new DecimalFormat("#.####");
                String roundedXPositionMeters = decimalFormat.format(xPosition);
                String roundedYPositionMeters =  decimalFormat.format(yPosition);
                String roundedZPositionMeters =  decimalFormat.format(zPosition);

                // Create a message that can be displayed on the image.
                String positionText = "(" + roundedXPositionMeters + "," + roundedYPositionMeters + "," +
                        roundedZPositionMeters + ")";

                // Display the object's estimated position on screen.
                Point positionLabelPosition = new Point(box.x - (box.width / 2.0), box.y + box.height);
                Imgproc.putText(outputFrame, positionText, positionLabelPosition, Imgproc.FONT_HERSHEY_DUPLEX,
                        0.8, new Scalar(0, 0, 0), 3); // Outline
                Imgproc.putText(outputFrame, positionText, positionLabelPosition, Imgproc.FONT_HERSHEY_DUPLEX,
                        0.8, new Scalar(255, 221, 51), 1); // Text
            }
        }

        telemetry.addLine("#Detected Objects");
    }

    /**
     * Checks if the newly detected sample has been seen before. If it has, update the previous detection. Otherwise,
     * add it io the list of detected objects.
     *
     * @param newSampleDetection The newly detected sample.
     */
    private void addToDetectedSamples(Sample newSampleDetection) {
        for (Sample sample : detectedObjects) {

            // If the sample's aren't the same type, skip to the next detection.
            if (newSampleDetection.getSampleType() != sample.getSampleType()) {
                continue;
            }

            // Check if the two samples are within the set distance tolerance identifying them as the same.
            if (!(Pose2d.getAbsolutePositionalDistanceTo(sample.getFieldPosition(), newSampleDetection.getFieldPosition()) <= sameObjectDistanceToleranceMeters)) {
                continue;
            }

            // The samples are the same, so update ethem.
            sample.update(newSampleDetection);
            return;
        }

        // The sample detection is new, so add it to the list of detected objects.
        this.detectedObjects.add(newSampleDetection);
    }

    /**
     * Estimates the given detected object's position in 3D space, assuming that the object is on the floor.
     *
     * @param detectedObject The bounding box around the detected object, as a Rect.
     */
    private void estimate3DPosition(Rect detectedObject) {

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

        // Represent the estimated position as a Mat object.
        this.objectWorldCoordinates = new Mat(1,3, CvType.CV_32FC1);
        this.objectWorldCoordinates.put(0, 0, xPositionMeters, -cameraDistanceFromGroundMeters, objectDepthMeters);
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
     * Sets this pipeline's current sample, which determines which color samples are detected. THis
     *
     * @param sampleType The type of the same that will be detected.
     */
    public void setCurrentSampleType(SampleType sampleType) {

        // Set this pipeline's current sample to the given sample type.
        this.currentSampleType = sampleType;

        // Set the color thresholds for pipeline based on the current sample.
        this.lowColorRange = this.currentSampleType.getLowHSVThreshold();
        this.highColorRange = this.currentSampleType.getHighHSVThreshold();
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

    /**
     * Returns a list of all of the detected objects.
     *
     * @return A list of all of the detected objects.
     */
    public List<Sample> getDetectedObjects() {
        return this.detectedObjects;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}