package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.opencv.core.Scalar;

public enum SampleType {
    RED ("RED", new Scalar(0,191,60), new Scalar(180,255,255)), // TODO: Tune
    BLUE ("BLUE", new Scalar(0, 0, 0), new Scalar(0, 0, 0)), // TODO: Tune
    NEUTRAL ("NEUTRAL", new Scalar(5,188,68), new Scalar(17,255,152));

    private final String sampleName;
    private final Scalar lowHSVThreshold;
    private final Scalar highHSVThreshold;
    SampleType(String sampleName, Scalar lowHSVThreshold, Scalar highHSVThreshold) {
        this.sampleName = sampleName;
        this.lowHSVThreshold = lowHSVThreshold;
        this.highHSVThreshold = highHSVThreshold;
    }

    /**
     * Return this SampleType's string equivalent / name.
     *
     * @return This SampleType's string equivalent / name.
     */
    public String getSampleName() {
        return sampleName;
    }

    /**
     * Returns this SampleType's low HSV threshold.
     *
     * @return This SampleType's low HSV threshold.
     */
    public Scalar getLowHSVThreshold() {
        return this.lowHSVThreshold;
    }

    /**
     * Returns this SampleType's high HSV threshold.
     *
     * @return This SampleType's high HSV threshold.
     */
    public Scalar getHighHSVThreshold() {
        return this.highHSVThreshold;
    }
}
