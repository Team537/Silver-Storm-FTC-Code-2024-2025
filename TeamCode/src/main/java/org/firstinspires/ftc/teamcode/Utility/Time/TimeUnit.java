package org.firstinspires.ftc.teamcode.Utility.Time;

public enum TimeUnit {
    SECOND (1000000000),
    MILLISECOND (1000000),
    MICROSECOND (1000);

    /**
     * Multiplier to convert this unit of time to nanoseconds.
     */
    public final long nanoSecondConversionMultiplier;
    TimeUnit(long nanoSecondConversionMultiplier) {
        this.nanoSecondConversionMultiplier = nanoSecondConversionMultiplier;
    }

    /**
     * Converts and returns the provided time in nanoseconds to this TimeUnit.
     *
     * @param timeNanoSeconds The time, in nanoseconds.
     * @return The time converted to this TimeUnit, as a double.
     */
    public double convertNanoSecondsTo(long timeNanoSeconds) {
        return (double) timeNanoSeconds / this.nanoSecondConversionMultiplier;
    }

    /**
     * Converts the given time from this TimeUnit to nanoseconds.
     *
     * @param time The time in this TimeUnit.
     * @return The time converted to nanoseconds.
     */
    public long convertToNanoSeconds(double time) {
        return (long) time * this.nanoSecondConversionMultiplier;
    }
}
