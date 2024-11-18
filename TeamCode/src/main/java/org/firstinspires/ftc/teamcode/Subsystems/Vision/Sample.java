package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.teamcode.Utility.Geometry.Vector;
import org.opencv.core.Rect;

public class Sample {
    private Vector robotRelativePosition;
    private SampleType sampleType;
    private Rect boundingBox;
    private double detectionTimeSeconds;

    /**
     * Create a new sample object with the given information.
     *
     * @param robotRelativePosition The 3D position of the sample.
     * @param sampleType The type of the sample.
     * @param boundingBox The bounding box of the object on the screen.
     */
    public Sample(Vector robotRelativePosition, SampleType sampleType, Rect boundingBox, double detectionTimeSeconds) {
        this.robotRelativePosition = robotRelativePosition;
        this.sampleType = sampleType;
        this.boundingBox = boundingBox;
        this.detectionTimeSeconds = detectionTimeSeconds;
    }

    /**
     * Return this Sample's robotRelativePosition.
     *
     * @return This Sample's robotRelativePosition.
     */
    public Vector getRobotRelativePosition() {
        return robotRelativePosition;
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
     * Return this Sample's boundingBox.
     *
     * @return This Sample's boundingBox.
     */
    public Rect getBoundingBox() {
        return boundingBox;
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
