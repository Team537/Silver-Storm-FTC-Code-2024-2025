package org.firstinspires.ftc.teamcode.Utility.Geometry;

import androidx.annotation.NonNull;

public class Rotation2d {

    private double pitch;
    private double yaw;

    /**
     * Create a new Rotation2d object with rotational values of 0.
     */
    public Rotation2d() {
        this.pitch = 0;
        this.yaw = 0;
    }

    /**
     * Create a Rotation2d object with the given rotational values.
     *
     * @param yawRadians The rotation along the Y Axis, in radians.
     * @param pitchRadians The rotation along the Z Axis, in radians.
     */
    public Rotation2d(double yawRadians, double pitchRadians)  {
        this.yaw = Rotation2d.clampAngle(yawRadians);
        this.pitch = Rotation2d.clampAngle(pitchRadians);
    }

    /**
     * Create a Rotation2d object with the rotational values of another Rotation2d.
     *
     * @param rotation2d The Rotation2d who's values will be clones to this Rotation2d.
     */
    public Rotation2d(Rotation2d rotation2d)  {
        this.yaw = Rotation2d.clampAngle(rotation2d.getYawInRadians());
        this.pitch = Rotation2d.clampAngle(rotation2d.getPitchInRadians());
    }

    /**
     * Clamps the given angle to be within the range of [-PI, PI]
     * <p> </p>
     * This ensures that the angle is normalized to a valid range in radians, where
     * -PI represents -180 degrees and PI represents 180 degrees. Any angle outside
     * this range is wrapped around to stay within [-PI, PI].
     *
     * @param angleToClamp The angle to be clamped, in radians.
     * @return The normalized angle, clamped within the range [-PI, PI].
     */
    static double clampAngle(double angleToClamp) {

        // Store a value that will contain the clamped angle. This is done to prevent directly
        // editing parameters.
        double clampedAngle = angleToClamp;

        // If the angle is greater than PI radians, wrap the angle back around into the negatives.
        while (clampedAngle > Math.PI) {
            clampedAngle -= (2 * Math.PI);
        }

        // If the angle is less than PI radians, wrap the angle back around into the negatives.
        while (clampedAngle < -Math.PI) {
            clampedAngle += (2 * Math.PI);
        }

        // Return the clamped angle.
        return clampedAngle;
    }


    /**
     * Returns a clone of this rotation2d.
     *
     * @return A clone of this rotation2d.
     */
    @NonNull
    public Rotation2d clone() {
        return new Rotation2d(this);
    }

    /**
     * Adds the values of another Rotation2d to this Rotation2d's values.
     *
     * @param rotationOffset The Rotation2d who's value will be added to this Rotation2d's values.
     */
    public void addValues(Rotation2d rotationOffset) {
        this.yaw = Rotation2d.clampAngle(this.yaw + rotationOffset.getYawInRadians());
        this.pitch = Rotation2d.clampAngle(this.pitch + rotationOffset.getPitchInRadians());
    }

    /**
     * Adds the provided values to this Rotation2d's values.
     *
     * @param yawOffsetRadians The yaw offset, it radians
     * @param pitchOffsetRadians The pitch offset, it radians
     */
    public void addValues(double yawOffsetRadians, double pitchOffsetRadians) {
        this.yaw = Rotation2d.clampAngle(this.yaw + yawOffsetRadians);
        this.pitch = Rotation2d.clampAngle(this.pitch + pitchOffsetRadians);
    }

    /**
     * Adds the given yaw offset to this Rotation2d's yaw.
     *
     * @param yawOffsetRadians The yaw offset, in radians.
     */
    public void addYaw(double yawOffsetRadians) {
        this.yaw = Rotation2d.clampAngle(this.yaw + yawOffsetRadians);
    }

    /**
     * Adds the given pitch offset to this Rotation2d's pitch.
     *
     * @param pitchOffsetRadians The pitch offset, in radians.
     */
    public void addPitch(double pitchOffsetRadians) {
        this.pitch = Rotation2d.clampAngle(this.pitch + pitchOffsetRadians);
    }

    /**
     * Subtracts the values of another Rotation2d from this Rotation2d's values.
     *
     * @param rotationOffset  The Rotation2d who's value will be subtracted from this Rotation2d's values.
     */
    public void subtractValues(Rotation2d rotationOffset ) {
        this.yaw = Rotation2d.clampAngle(this.yaw - rotationOffset.getYawInRadians());
        this.pitch = Rotation2d.clampAngle(this.pitch - rotationOffset.getPitchInRadians());
    }

    /**
     * Subtracts the provided values from this Rotation2d's values.
     *
     * @param yawOffsetRadians The yaw offset, it radians
     * @param pitchOffsetRadians The pitch offset, it radians
     */
    public void subtractValues(double yawOffsetRadians, double pitchOffsetRadians) {
        this.yaw = Rotation2d.clampAngle(this.yaw - yawOffsetRadians);
        this.pitch = Rotation2d.clampAngle(this.pitch - pitchOffsetRadians);
    }

    /**
     * Subtracts the given yaw offset from this Rotation2d's yaw.
     *
     * @param yawOffsetRadians The yaw offset, in radians.
     */
    public void subtractYaw(double yawOffsetRadians) {
        this.yaw = Rotation2d.clampAngle(this.yaw - yawOffsetRadians);
    }

    /**
     * Subtracts the given pitch offset from this Rotation2d's pitch.
     *
     * @param pitchOffsetRadians The pitch offset, in radians.
     */
    public void subtractPitch(double pitchOffsetRadians) {
        this.pitch = Rotation2d.clampAngle(this.pitch - pitchOffsetRadians);
    }

    /**
     * Sets the yaw value of this Rotation2d.
     *
     * @param yawRadians The rotation along the Y Axis, in radians.
     */
    public void setYaw(double yawRadians) {
        this.yaw = clampAngle(yawRadians);
    }

    /**
     * Sets the pitch value of this Rotation2d.
     *
     * @param pitchRadians The rotation along the Z Axis, in radians.
     */
    public void setPitch(double pitchRadians) {
        this.pitch = clampAngle(pitchRadians);
    }

    /**
     * Sets the values of this Rotation2d.
     *
     * @param yawRadians The rotation along the Y Axis, in radians.
     * @param pitchRadians The rotation along the Z Axis, in radians.
     */
    public void setValues(double yawRadians, double pitchRadians) {
        this.yaw = Rotation2d.clampAngle(yawRadians);
        this.pitch = Rotation2d.clampAngle(pitchRadians);
    }


    /**
     * Sets the values of this Rotation2d.
     *
     * @param rotation2d The Rotation2d who's value will be clones to this Rotation2d.
     */
    public void setValues(Rotation2d rotation2d) {
        this.yaw = Rotation2d.clampAngle(rotation2d.getYawInRadians());
        this.pitch = Rotation2d.clampAngle(rotation2d.getPitchInRadians());
    }


    /**
     * Returns this Rotation2d object's rotation along the Y axis, in radians.
     *
     * @return This Rotation2d object's rotation along the Y axis, in radians.
     */
    public double getYawInRadians() {
        return this.yaw;
    }

    /**
     * Returns this Rotation2d object's rotation along the Y axis, in degrees.
     *
     * @return This Rotation2d object's rotation along the Y axis, in degrees.
     */
    public double getYawInDegrees() {
        return Math.toDegrees(this.yaw);
    }

    /**
     * Returns this Rotation2d object's rotation along the Z axis, in radians.
     *
     * @return This Rotation2d object's rotation along the Z axis, in radians.
     */
    public double getPitchInRadians() {
        return this.pitch;
    }

    /**
     * Returns this Rotation2d object's rotation along the Z axis, in degrees.
     *
     * @return This Rotation2d object's rotation along the Z axis, in degrees.
     */
    public double getPitchInDegrees() {
        return Math.toDegrees(this.pitch);
    }
}