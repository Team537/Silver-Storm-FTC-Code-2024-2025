package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Constants.VisionConstants.LogitechBrio100Constants;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Storage.FileEx;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class ComputerVision implements Subsystem {

    private Function<Pose2d, Pose2d> robotToFieldSpaceConversion;
    private OpenCvWebcam camera;
    private SampleDetectionPipeline neutralSamplePipeline;
    private FileEx cameraSettings;

    // Settings
    private final String CAMERA_SETTINGS_FILE_NAME = "CameraSettings.txt";
    private final String CAMERA_EXPOSURE_VALUE_NAME = "cameraExposure";
    private final String CAMERA_GAIN_VALUE_NAME = "cameronGain";
    private final long DEFAULT_EXPOSURE_TIME_MILLISECONDS = 5;
    private final int DEFAULT_GAIN = 50;

    public ComputerVision(Function<Pose2d, Pose2d> robotToFieldSpaceConversion) {
        this.robotToFieldSpaceConversion = robotToFieldSpaceConversion;
    }

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Get access to the camera's settings.
        cameraSettings = new FileEx(CAMERA_SETTINGS_FILE_NAME);

        // Setup the neutralSamplePipeline
        neutralSamplePipeline = new SampleDetectionPipeline(LogitechBrio100Constants.STREAM_WIDTH_PIXELS,
                LogitechBrio100Constants.STREAM_HEIGHT_PIXELS, telemetry, robotToFieldSpaceConversion);
        neutralSamplePipeline.setDistortionCoefficients(LogitechBrio100Constants.DISTORTION_COEFFICIENTS);
        neutralSamplePipeline.setIntrinsicCameraMatrix(LogitechBrio100Constants.CAMERA_MATRIX);
        neutralSamplePipeline.setExtrinsicCameraMatrix(LogitechBrio100Constants.CAMERA_ROTATION_MATRIX, LogitechBrio100Constants.CAMERA_TRANSLATION_MATRIX);
        neutralSamplePipeline.setCurrentSampleType(SampleType.BLUE);
        
        // Setup the camera
        setupCamera(hardwareMap);
    }

    /**
     * Sets up the camera so that various vision operations may be preformed.
     *
     * @param hardwareMap The opMode's hardware map.
     */
    private void setupCamera(HardwareMap hardwareMap) {

        // Create the physical camera object.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        // Setup the camera and start streaming video feed to the driver station.
        camera.openCameraDevice(); // TODO: Figure out how to use the non-deprecated method.

        // Attempt to get the saved camera exposure and gain from the file. The class non-primate data
        // types are used in case no data exists and null is returned, which ensures no errors occur.
        Long cameraExposureTimeMilliseconds = cameraSettings.getValue(CAMERA_EXPOSURE_VALUE_NAME, Long.class);
        Integer cameraGain = cameraSettings.getValue(CAMERA_GAIN_VALUE_NAME, Integer.class);

        // If there isn't any saved data for the camera's exposure or gain, use the default values.
        if (cameraExposureTimeMilliseconds == null) {
            cameraExposureTimeMilliseconds = DEFAULT_EXPOSURE_TIME_MILLISECONDS;
        }
        if (cameraGain == null) {
            cameraGain = DEFAULT_GAIN;
        }

        // Set the exposure of the camera. This ensures that
        ExposureControl cameraExposureControl = camera.getExposureControl();
        cameraExposureControl.setMode(ExposureControl.Mode.Manual);
        cameraExposureControl.setExposure(cameraExposureTimeMilliseconds, TimeUnit.MILLISECONDS);

        GainControl cameraGainControl = camera.getGainControl();
        cameraGainControl.setGain(cameraGain);

        // Set the pipeline of the camera and start streaming.
        camera.setPipeline(neutralSamplePipeline);
        camera.startStreaming(LogitechBrio100Constants.STREAM_WIDTH_PIXELS,
                LogitechBrio100Constants.STREAM_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * Returns a list of detected objects. This includes objects not currently visible.
     *
     * @return A list of detected objects.
     */
    public List<Sample> getDetectedObjects() {
        return this.neutralSamplePipeline.getDetectedObjects();
    }

    /**
     * Saves the current camera exposure and gian to the file so that it can be loaded at a later date.
     */
    public void saveCameraSettings() {

        // Get the camera exposure and gain.
        long cameraExposure = getExposure();
        int cameraGain = getGain();

        // Write the values to the file.
        cameraSettings.addData(CAMERA_EXPOSURE_VALUE_NAME, cameraExposure);
        cameraSettings.addData(CAMERA_GAIN_VALUE_NAME, cameraGain);

        // Save the file.
        cameraSettings.saveData();
    }

    /**
     * Take a photo using all cameras.
     */
    public void captureFrame() {
        neutralSamplePipeline.captureFrame();
    }

    /**
     * Set whether or not this pipeline will record new vision data.
     *
     * @param pauseCapture Whether or not this pipeline will record new vision data.
     */
    public void setPauseCapture(boolean pauseCapture) {
        this.neutralSamplePipeline.setPauseCapture(pauseCapture);
    }

    /**
     * Sets the exposure of the active camera to the given value.
     *
     * @param exposureTimeMilliseconds The exposure time, in milliseconds.
     */
    public void setExposure(long exposureTimeMilliseconds) {

        // Get the exposure controller from the active camera and set the exposure to the given value.
        ExposureControl cameraExposureControl = camera.getExposureControl();
        cameraExposureControl.setExposure(exposureTimeMilliseconds, TimeUnit.MILLISECONDS);
    }


    /**
     * Sets the gain of the active camera to the given value.
     *
     * @param gain The gain to set the active camera to.
     */
    public void setGain(int gain) {

        // Get the gain controller from the active camera and set the gain to the given value.
        GainControl camerGainControl = camera.getGainControl();
        camerGainControl.setGain(gain);
    }

    /**
     * Returns the exposure time of the active camera in milliseconds.
     *
     * @return The exposure time of the active camera in milliseconds.
     */
    public long getExposure() {

        // Get the gain controller from the active camera.
        ExposureControl cameraExposureControl = camera.getExposureControl();

        // Return the current exposure value.
        return cameraExposureControl.getExposure(TimeUnit.MILLISECONDS);
    }

    /**
     * Returns the gain of the active camera.
     *
     * @return The gain of the active camera.
     */
    public int getGain() {

        // Get the gain controller from the active camera.
        GainControl camerGainControl = camera.getGainControl();

        // Return the camera's gain.
        return camerGainControl.getGain();
    }

    /**
     * Returns this Computer Vision's sample pipeline.
     *
     * @return This Computer Vision's sample pipeline.
     */
    public SampleDetectionPipeline getSampleDetectionPipeline() {
        return this.neutralSamplePipeline;
    }

    /**
     * Returns the runtime of this pipeline.
     *
     * @return The runtime of this pipeline.
     */
    public double getVisionTimeSeconds() {
        return this.neutralSamplePipeline.getVisionTimeSeconds();
    }

    @Override
    public void periodic() {

    }
}
