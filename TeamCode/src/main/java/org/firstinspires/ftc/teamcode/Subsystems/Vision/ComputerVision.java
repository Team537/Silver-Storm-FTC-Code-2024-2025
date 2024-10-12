package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Constants.VisionConstants.LogitechBrio100Constants;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

public class ComputerVision implements Subsystem {

    private OpenCvWebcam camera;
    private GenericSamplePipeline neutralSamplePipeline;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup the neutralSamplePipeline
        neutralSamplePipeline = new GenericSamplePipeline(LogitechBrio100Constants.STREAM_WIDTH_PIXELS,
                LogitechBrio100Constants.STREAM_HEIGHT_PIXELS, telemetry);
        neutralSamplePipeline.setDistortionCoefficients(LogitechBrio100Constants.DISTORTION_COEFFICIENTS);
        neutralSamplePipeline.setIntrinsicCameraMatrix(LogitechBrio100Constants.CAMERA_MATRIX);
        neutralSamplePipeline.setExtrinsicCameraMatrix(LogitechBrio100Constants.CAMERA_ROTATION_MATRIX, LogitechBrio100Constants.CAMERA_TRANSLATION_MATRIX);

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

        // Set the exposure of the camera. This ensures that
        ExposureControl cameraExposureControl = camera.getExposureControl();
        cameraExposureControl.setMode(ExposureControl.Mode.Manual);
        cameraExposureControl.setExposure(50, TimeUnit.MILLISECONDS);

        GainControl cameraGainControl = camera.getGainControl();
        cameraGainControl.setGain(100);

        // Set the pipeline of the camera and start streaming.
        camera.setPipeline(neutralSamplePipeline);
        camera.startStreaming(LogitechBrio100Constants.STREAM_WIDTH_PIXELS,
                LogitechBrio100Constants.STREAM_HEIGHT_PIXELS, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * Take a photo using all cameras.
     */
    public void captureFrame() {
        neutralSamplePipeline.captureFrame();
    }

    @Override
    public void periodic() {

    }
}
