package org.firstinspires.ftc.teamcode.Utility.Time;

public class ElapsedTime {

    private long startTimeNanoseconds;

    /**
     * Constructs a new ElapsedTime instance and uses the current system time, in nanoseconds, as
     * the starting time.
     */
    public ElapsedTime() {
        this.startTimeNanoseconds = System.nanoTime();
    }

    /**
     * Creates a new ElapsedTime object with the specified starting time, in nanoseconds.
     * @param startTimeNanoseconds The starting time of this ElapsedTime object, in nanoseconds.
     */
    public ElapsedTime(long startTimeNanoseconds) {
        this.startTimeNanoseconds = startTimeNanoseconds;
    }


    /**
     * Creates a new ElapsedTime object using the provided starting time in the specified unit.
     *
     * @param startingTime The starting time, in the specified unit.
     * @param timeUnit The unit of the provided time value.
     */
    public ElapsedTime(double startingTime, TimeUnit timeUnit) {
        this.startTimeNanoseconds = timeUnit.convertToNanoSeconds(startingTime);
    }

    /**
     * Resets this ElapsedTime's starting time to the current point in time, in nanoseconds.
     */
    public void reset() {
        this.startTimeNanoseconds = System.nanoTime();
    }

    /**
     * Returns the amount of time that has passed since this ElapsedTime object's starting time in nanoseconds.
     *
     * @return The elapsed time since this ElapsedTime object's starting time, in nanoseconds.
     */
    public long getElapsedTime() {
        return System.nanoTime() - startTimeNanoseconds;
    }

    /**
     * Returns the amount of time that has passed since this ElapsedTime object's starting time.
     *
     * @param unit The TimeUnit in which to return the elapsed time.
     * @return The elapsed time since this ElapsedTime object's starting time, converted to the specified TimeUnit.
     */
    public double getElapsedTime(TimeUnit unit) {

        // Get the elapsed time in nanoseconds.
        long elapsedTimeNanoseconds = System.nanoTime() - startTimeNanoseconds;

        // Return the value converted to the
        return unit.convertNanoSecondsTo(elapsedTimeNanoseconds);
    }

    /**
     * Returns the elapsed time since the provided time, in nanoseconds.
     *
     * @param timeNanoseconds The time that will be used to calculate elapsed timm, in nanoseconds.
     * @return The elapsed time since the provided time, in nanoseconds.
     */
    public long getTimeSince(long timeNanoseconds) {
        return System.nanoTime() - timeNanoseconds;
    }

    /**
     * Returns the elapsed time since the provided time, in nanoseconds.
     *
     * @param time The time that will be used to calculate elapsed time, in the specified TimeUnit.
     * @param timeUnit The TimeUnit of the provided time value.
     * @return The elapsed time since the provided time, in nanoseconds.
     */
    public long getTimeSince(double time, TimeUnit timeUnit) {
        return getTimeSince(timeUnit.convertToNanoSeconds(time));
    }

    /**
     * Returns the elapsed time since the provided time, in the specified TimeUnit.
     *
     * @param timeNanoseconds The time that will be used to calculate elapsed time, in nanoseconds.
     * @param outputUnit The TimeUnit in which to return the elapsed time.
     * @return  The elapsed time since the provided time, in the specified TimeUnit.
     */
    public double getTimeSince(long timeNanoseconds, TimeUnit outputUnit) {
        return outputUnit.convertNanoSecondsTo(getTimeSince(timeNanoseconds));
    }

    /**
     * Returns the elapsed time since the provided time, in the specified TimeUnit.
     *
     * @param time The time that will be used to calculate elapsed time, in the specified TimeUnit.
     * @param timeUnit The TimeUnit of the provided time value.
     * @param outputUnit The TimeUnit in which to return the elapsed time.
     * @return The elapsed time since the provided time, in the specified TimeUnit.
     */
    public double getTimeSince(double time, TimeUnit timeUnit, TimeUnit outputUnit) {
        return outputUnit.convertNanoSecondsTo(getTimeSince(timeUnit.convertToNanoSeconds(time)));
    }
}
