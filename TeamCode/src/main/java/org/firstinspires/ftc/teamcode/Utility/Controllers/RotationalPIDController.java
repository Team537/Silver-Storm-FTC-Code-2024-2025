package org.firstinspires.ftc.teamcode.Utility.Controllers;

import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;

public class RotationalPIDController extends PIDController {

    /**
     * Create a new PIDController object with empty PID coefficients.
     */
    public RotationalPIDController() {
        super();
    }

    /**
     * Create a new PIDController object with the given PID coefficients.
     * @param kp Helps prevents overshooting the target
     * @param kd Help prevent oscillation (Value changing rapidly back and forth).
     * @param ki Helps prevent steady state error. (Helps overcome obstacles are consistently
    blocking it off.
     */
    public RotationalPIDController(double kp, double kd, double ki) {
        super(kp, ki, kd);
    }

    @Override
    public double update(double targetValue, double currentValue) {

        // Calculate the error between our current value and our target value and clamp the angular value.
        double error = targetValue - currentValue;
        error = clampAngle(error);

        // Add our current error * delta time to our accumulated error. This helps us visualize how much error is
        // accumulated over time.
        this.accumulatedError += (error * elapsedTime.getElapsedTime(TimeUnit.SECOND));

        // Compensate for if it is more efficient to go in the other direction.
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

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

    /**
     * Clamp the given angle to within the given bounds.
     *
     * @param angleRadians The angle that needs to be clamped, in radians.
     * @return The clamped angle.
     */
    private double clampAngle(double angleRadians) {
        double outputAngleRadians = angleRadians;
        while (outputAngleRadians <= -Math.PI) {
            outputAngleRadians += 2 * Math.PI;
        }
        while (outputAngleRadians > Math.PI) {
            outputAngleRadians -= 2 * Math.PI;
        }
        return outputAngleRadians;
    }
}
