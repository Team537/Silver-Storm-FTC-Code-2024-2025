package org.firstinspires.ftc.teamcode.Utility.Geometry;

import androidx.annotation.NonNull;

import org.opencv.core.Mat;

public class Pose2d {
    private double x;
    private double z;
    private Rotation2d rotation2d;

    /**
     * Creates an empty Pose2d object.
     */
    public Pose2d() {
        this.x = 0;
        this.z = 0;
        this.rotation2d = new Rotation2d();
    }

    /**
     * Creates a Pose2d object with the given positional values and a blank rotation.
     *
     * @param xMeters The x position, in meters.
     * @param zMeters The z position, in meters.
     */
    public Pose2d(double xMeters, double zMeters) {
        this.x = xMeters;
        this.z = zMeters;
        this.rotation2d = new Rotation2d();
    }

    /**
     * Creates a new Pose2d object using the given values.
     *
     * @param xMeters The x position, in meters.
     * @param zMeters The z position, in meters.
     * @param rotation2d The rotation of the position, as a Rotation2d.
     */
    public Pose2d(double xMeters, double zMeters, Rotation2d rotation2d) {
        this.x = xMeters;
        this.z = zMeters;
        this.rotation2d = rotation2d;
    }

    /**
     * Create a Pose2d object with the rotational values of another Pose2d.
     *
     * @param pose2d The Pose2d who's values will be clones to this Pose2d.
     */
    public Pose2d(Pose2d pose2d)  {
       this.x = pose2d.getX();
       this.z = pose2d.getZ();
       this.rotation2d = pose2d.getRotation2d();
    }

    /**
     * Calculates and returns the distance from this Pose2d to the provided targetPosition, as a Pose2d.
     *
     * @param targetPosition The position you wish to find the distance to.
     * @return Returns the distance to the provided point, as a Pose2d.
     */
    public Pose2d getDistanceTo(Pose2d targetPosition) {

        // Calculate the distance between each point on each axis.
        double xDistanceMeters = targetPosition.getX() - this.x;
        double zDistanceMeters = targetPosition.getZ() - this.z;

        // Calculate the rotational different between this position and the target.
        Rotation2d rotationalDifference = targetPosition.getRotation2d();
        rotationalDifference.subtractValues(this.rotation2d);

        // Return the distance between the two points as a Pose2d.
        return new Pose2d(xDistanceMeters, zDistanceMeters, rotationalDifference);
    }

    /**
     * Calculates and returns the distance from this Pose2d to the provided targetPosition, as a double.
     *
     * @param targetPosition The position you wish to find the distance to.
     * @return The distance to the provided point, as a double.
     */
    static double getAbsolutePositionalDistanceTo(Pose2d originPosition, Pose2d targetPosition) {

        // Calculate the distance between each point on each axis.
        double xDistanceMeters = originPosition.getX() - targetPosition.getX();
        double zDistanceMeters = originPosition.getZ() - targetPosition.getZ();

        // Return the distance between the two points.
        // This is calculated using the distance formula: d = √(x2-x1)^2 + (z2-z1)^2
        return Math.sqrt(Math.pow(xDistanceMeters, 2) + Math.pow(zDistanceMeters, 2));
    }

    /**
     * Returns a clone of this Pose2d.
     *
     * @return A clone of this Pose2d.
     */
    @NonNull
    public Pose2d clone() {
        return new Pose2d(this);
    }

    /**
     * Adds the values from another Pose2d to this Pose2d's values.
     *
     * @param positionalOffset The pose2d who's values will be added to this Pose2d's's values.
     */
    public void addValues(Pose2d positionalOffset) {
        this.x += positionalOffset.getX();
        this.z += positionalOffset.getZ();
        this.rotation2d.addValues(positionalOffset.getRotation2d());
    }

    /**
     * Subtract the values from another Pose2d from this Pose2d's values.
     *
     * @param positionalOffset The pose2d who's values will be subtracted from this Pose2d's's values.
     */
    public void subtractValues(Pose2d positionalOffset) {
        this.x -= positionalOffset.getX();
        this.z -= positionalOffset.getZ();
        this.rotation2d.subtractValues(positionalOffset.getRotation2d());
    }

    /**
     * Sets this Pose2d's x position to the provided value.
     *
     * @param xMeters The x position, in meters.
     */
    public void setX(double xMeters) {
        this.x = xMeters;
    }

    /**
     * Sets this Pose2d's z position to the provided value.
     *
     * @param zMeters The z position, in meters.
     */
    public void setZ(double zMeters) {
        this.z = zMeters;
    }

    /**
     * Sets this Pose2d's rotation2d to the provided object.
     *
     * @param rotation2d The rotation, as a Rotation2d.
     */
    public void setRotation(Rotation2d rotation2d) {
        this.rotation2d = rotation2d;
    }

    /**
     * Returns this Pose2d's x value, in meters.
     *
     * @return This Pose2d's x value, in meters.
     */
    public double getX() {
        return this.x;
    }

    /**
     * Returns this Pose2d's z value, in meters.
     *
     * @return This Pose2d's z value, in meters.
     */
    public double getZ() {
        return this.z;
    }

    /**
     * Returns the magnitude of this Pose2d.
     *
     * @return The magnitude of this Pose2d.
     */
    public double getMagnitude() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.z, 2));
    }

    /**
     * Returns a clone of this Pose2d's rotation2d.
     *
     * @return A clone of this Pose2d's rotation2d.
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(this.rotation2d);
    }
}
