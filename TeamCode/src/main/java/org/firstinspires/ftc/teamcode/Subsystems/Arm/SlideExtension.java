package org.firstinspires.ftc.teamcode.Subsystems.Arm;

public enum SlideExtension {
    REST (0),
    QUARTER (0.085725),
    HALF (0.17145),
    THREE_QUARTERS (0.257175),
    FULL (0.3429);

    private final double EXTENSION_LENGTH_METERS;
    SlideExtension(double extensionLengthMeters) {
        this.EXTENSION_LENGTH_METERS = extensionLengthMeters;
    }

    /**
     * Return this slide extension length's extension distance in enters.
     *
     * @return This slide extension length's extension distance in enters.
     */
    public double getExtensionLengthMeters() {
        return this.EXTENSION_LENGTH_METERS;
    }
}
