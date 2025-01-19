package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Vector;
import org.opencv.core.Rect;

public class Sample {
    private Pose2d fieldPosition;
    private SampleType sampleType;
    private double detectionTimeSeconds;

    /**
     * Create a new sample object with the given information.
     *
     * @param fieldPosition The 3D position of the sample.
     * @param sampleType The type of the sample.
     */
    public Sample(Pose2d fieldPosition, SampleType sampleType, double detectionTimeSeconds) {
        this.fieldPosition = fieldPosition;
        this.sampleType = sampleType;
        this.detectionTimeSeconds = detectionTimeSeconds;
    }

    /**
     * If a new detection of this sample is found, update this sample's data.
     *
     * @param updatedSample The new detection of this sample.
     */
    public void update(Sample updatedSample) {
        this.fieldPosition = updatedSample.getFieldPosition();
        this.detectionTimeSeconds = updatedSample.getDetectionTimeSeconds();
    }

    /**
     * Return this Sample's robotRelativePosition.
     *
     * @return This Sample's robotRelativePosition.
     */
    public Pose2d getFieldPosition() {
        return fieldPosition;
    }

    /**
     * Return this Sample's sampleType.
     *
     * @return This Sample's sampleType.
     */
    public SampleType getSampleType() {
        return sampleType;
    }

    /**
     * Return the time this sample was detected in seconds.
     *
     * @return The time this sample was detected in seconds.
     */
    public double getDetectionTimeSeconds() {
        return detectionTimeSeconds;
    }
}
