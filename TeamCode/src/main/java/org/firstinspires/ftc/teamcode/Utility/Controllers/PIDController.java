package org.firstinspires.ftc.teamcode.Utility.Controllers;

import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

public class PIDController {

    // PID Coefficients
    protected double kp;
    protected double ki;
    protected double kd;
    protected double errorTolerance;

    // Storage
    protected double accumulatedError;
    protected double previousError;
    protected double lastTarget;
    protected ElapsedTime elapsedTime;

    /**
     * Create a new PIDController object with empty PID coefficients.
     */
    public PIDController() {

        // Set all PID coefficients to 0, since no values were provided.
        this.kp = 0;
        this.ki = 0;
        this.kd = 0;

        // Setup the Elapsed Time object.
        this.elapsedTime = new ElapsedTime();

        // Setup the error values.
        this.reset();
    }

    /**
     * Create a new PIDController object with the given PID coefficients.
     * @param kp Helps prevents overshooting the target
     * @param kd Help prevent oscillation (Value changing rapidly back and forth).
     * @param ki Helps prevent steady state error. (Helps overcome obstacles are consistently
                 blocking it off.
     */
    public PIDController(double kp, double kd, double ki) {

        // Set all PID coefficients to the given values.
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        // Setup the Elapsed Time object.
        this.elapsedTime = new ElapsedTime();

        // Setup the error values.
        this.reset();
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    /**
     * Reset all of the PIDControllers values.
     */
    public void reset() {
        this.accumulatedError = 0;
        this.previousError = 0;
        this.lastTarget = 0;
        this.elapsedTime.reset();
    }

    public void setErrorTolerance(double errorTolerance) {
        this.errorTolerance = errorTolerance;
    }

    public double update(double targetValue, double currentValue) {

        // Calculate the error between our current value and our target value.
        double error = targetValue - currentValue;

        // Add our current error * delta time to our accumulated error. This helps us visualize how much error is
        // accumulated over time.
        this.accumulatedError += (error * elapsedTime.getElapsedTime(TimeUnit.SECOND));

        // Compensate for if it is more efficient to go in the other direction.
        // accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // Calculate how quickly the error is increasing / decreasing.
        double errorRateOfChange = (error - previousError) / elapsedTime.getElapsedTime(TimeUnit.SECOND);
        previousError = error;

        // If we are within the allowed error tolerance, then remove the accumulated error.
        // This helps prevent constant oscillations, since we will almost never reach the exact target value.
        if (Math.abs(error) < errorTolerance) {
            accumulatedError = 0;
            return 0;
        }

        // If we change targets, remove the accumulated error. This makes it easier to change directions
        // with certain mechanisms.
        if (targetValue != lastTarget) {
            accumulatedError = 0;
        }

        // Set the current target as the last target.
        lastTarget = targetValue;

        // Reset the timer for the next usage. Enables kore accurate delta time.
        elapsedTime.reset();

        // Calculate and return the result.
        return Math.tanh(kp * error + (ki * accumulatedError) + (kd * errorRateOfChange));
    }

}
