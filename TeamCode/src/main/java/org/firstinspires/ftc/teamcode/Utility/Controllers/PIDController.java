package org.firstinspires.ftc.teamcode.Utility.Controllers;

import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

public class PIDController {

    // PID Coefficients
    private double kp;
    private double ki;
    private double kd;
    private double feedForwards;

    private double errorTolerance;
    // Storage
    private double accumulatedError;
    private double previousError;
    private double lastTarget;
    private ElapsedTime elapsedTime;

    /**
     * Create a new PIDController object with empty PID coefficients.
     */
    public PIDController() {

        // Set all PID coefficients to 0, since no values were provided.
        this.kp = 0;
        this.ki = 0;
        this.kd = 0;

        // Setup the Elapsed Time object.
        elapsedTime = new ElapsedTime();

        // Setup the error values.
        this.reset();
    }

    /**
     * Create a new PIDController object with the given PID coefficients.
     * @param kp Helps prevents overshooting the target
     * @param ki Helps prevent steady state error. (Helps overcome obstacles are consistently
     *           blocking it off.
     * @param kd Help prevent oscillation (Value changing rapidly back and forth).
     */
    public PIDController(double kp, double kd, double ki) {

        // Set all PID coefficients to the given values.
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        // Setup the Elapsed Time object.
        elapsedTime = new ElapsedTime();

        // Setup the error values.
        this.reset();
    }

    /**
     * Reset all of the PIDControllers values.
     */
    public void reset() {
        this.accumulatedError = 0;
        this.previousError = 0;
        this.lastTarget = -0.12;
        elapsedTime.reset();
    }

    public double update(double targetValue, double currentValue) {

        // Calculate the error between our current value and our target value.
        double error = targetValue - currentValue;

        // Add our current error * delta time to our accumulated error. This helps us visualize how much error is
        // accumulated over time.
        accumulatedError += (error * elapsedTime.getElapsedTime(TimeUnit.SECOND));

        // Compensate for if it is more efficient to go in the other direction.
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // Calculate how quickly the error is increasing / decreasing.
        double errorRateOfChange = (error - previousError) / elapsedTime.getElapsedTime(TimeUnit.SECOND);
        previousError = error;

        // If we are within the allowed error tolerance, then remove the accumulated error.
        // This helps prevent constant oscillations, since we will almost enver reach the exact target value.
        if (Math.abs(error) < errorTolerance) {
            accumulatedError = 0;
        }


        // If we change targets, remove the accumulated error. This makes it easier to change directions
        // with certain mechanisms.
        if (lastTarget != -0.12 && targetValue != lastTarget) {
            accumulatedError = 0;
        }

        // Set the current target as the last target.
        lastTarget = targetValue;

        // Calculate and return the result.
        return Math.tanh( kp * error + (ki * accumulatedError) + (kd * errorRateOfChange));
    }

}
