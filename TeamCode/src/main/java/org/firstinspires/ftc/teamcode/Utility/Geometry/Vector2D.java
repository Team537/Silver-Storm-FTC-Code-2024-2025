package org.firstinspires.ftc.teamcode.Utility.Geometry;

public class Vector2D extends Vector {

    /**
     * Constructs a new 2D vector with the specified x and y coordinates.
     * This class is a specialized version of the base vector class that
     * specifically represents a vector in 2D space.
     *
     * @param x The x-coordinate (first component) of the 2D vector.
     * @param y The y-coordinate (second component) of the 2D vector.
     */
    public Vector2D(double x, double y) {
        super(new double[]{x, y});
    }

    /**
     * Rotates this Vector2D rotated by the given angle and returns the result as a new Vector2D.
     *
     * @param angleInRadians The angle that this Vector will be rated by, in radians.
     * @return This Vector2D rotated by the given angle and returns the result as a new Vector2D.
     */
    public Vector2D rotateVectorBy(double angleInRadians) {

        // Get this Vector2D's X and Y values.
        double x = this.VALUES[0];
        double y = this.VALUES[1];

        // Get the rotated coordinates of this Vector.
        double rotatedX = (x * Math.cos(angleInRadians)) - (y * Math.sin(angleInRadians));
        double rotatedY = (x * Math.sin(angleInRadians)) + (y * Math.cos(angleInRadians));

        // Return a new Vector2D with the rotated coordinates.
        return new Vector2D(x, y);
    }
}
