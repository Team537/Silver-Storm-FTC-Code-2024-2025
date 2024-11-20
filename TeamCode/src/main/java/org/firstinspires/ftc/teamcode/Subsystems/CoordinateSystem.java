package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CoordinateSystem implements Subsystem {

    // Settings
    private final double TICKS_TO_METERS = 0;

    // Hardware Storage
    private DcMotor fbOdometryPod; // X Coordinate
    private DcMotor lrOdometryPod; // Z Coordinate

    // Storage Coordinate Data
    private double robotXCoordinate = 0;
    private double robotZCoordinate = 0;

    private double lastXPosition = 0;
    private double lastZPosition = 0;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup hardware.
        setupHardware(hardwareMap);
    }

    /**
     * Sets up the robot's hardware.
     *
     * @param hardwareMap The opMode's hardware map. This is required in order to gian access
     *                    to the robot's hardware.
     */
    private void setupHardware(@NonNull HardwareMap hardwareMap) {

        // Get the "motors" from the hardware map and store them in variables. This is how we have to
        // access encoders.
        this.fbOdometryPod = hardwareMap.get(DcMotor.class, "fbOdPod");
        this.lrOdometryPod = hardwareMap.get(DcMotor.class, "lrOdPod");
        
    }

    private void updatePosition() {

        double currentXPosition = fbOdometryPod.getCurrentPosition();
        double currentZPosition = lrOdometryPod.getCurrentPosition();

        double xChange = currentXPosition - lastXPosition;
        double zChange = currentZPosition - lastZPosition;

        // TODO: Translate to origin?
    }

    @Override
    public void periodic() {
        updatePosition();
    }
}
