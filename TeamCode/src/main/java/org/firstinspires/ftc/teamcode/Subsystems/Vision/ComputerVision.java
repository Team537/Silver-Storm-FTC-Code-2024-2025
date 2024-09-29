package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ComputerVision implements Subsystem {

    private OpenCvCamera camera;
    private VisionPortal visionPortal;
    private GenericSamplePipeline neutralSamplePipeline;
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup the neutralSamplePipeline
        neutralSamplePipeline = new GenericSamplePipeline();

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
        camera.setPipeline(neutralSamplePipeline);
        camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void periodic() {

    }
}
