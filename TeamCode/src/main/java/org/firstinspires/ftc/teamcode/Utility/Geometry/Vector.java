package org.firstinspires.ftc.teamcode.Utility.Geometry;

import java.util.stream.IntStream;

public class Vector {

    // Storage
    protected final double[] VALUES;
    protected final int SIZE;
    protected double magnitude;

    // Flags
    private boolean isMagnitudeCalculated = false;

    /**
     * Creates a new Vector with the given values.
     *
     * @param vectorValues The values that this Vector will contain.
     */
    public Vector(double[] vectorValues) {
        this.VALUES = vectorValues;
        this.SIZE = vectorValues.length;
    }

    /**
     * Adds the values of the provided Vector to this Vector's values and returns the result as a new Vector.
     * The two Vectors must have the same number of elements, otherwise an exception is thrown.
     *
     * @param otherVector The Vector whose values will be added to this Vector.
     * @return A new Vector containing the element-wise sum of this Vector and the provided Vector.
     * @throws IllegalArgumentException If the provided Vector does not contain the same number of elements as this Vector.
     */
    public Vector add(Vector otherVector) {

        // Make sure that both Vectors contain the same number of values.
        if (otherVector.getSize() != this.SIZE) {
            throw new IllegalArgumentException("Cannot add two vectors that do not contain the same number of elements.");
        }

        // Calculate the sum of both Vectors.
        double[] resultVectorValue = new double[this.SIZE];
        for (int i = 0; i < this.SIZE; i++) {
            resultVectorValue[i] = this.VALUES[i] + otherVector.getValueAt(i);
        }

        // Return the result as a new Vector.
        return new Vector(resultVectorValue);
    }

    /**
     * Subtracts the values of the provided Vector from this Vector's values and returns the result as a new Vector.
     * The two Vectors must have the same number of elements, otherwise an exception is thrown.
     *
     * @param otherVector The Vector whose values will be subtracted from this Vector.
     * @return A new Vector containing the element-wise difference between this Vector and the provided Vector.
     * @throws IllegalArgumentException If the provided Vector does not contain the same number of elements as this Vector.
     */
    public Vector subtract(Vector otherVector) {

        // Make sure that both Vectors contain the same number of values.
        if (otherVector.getSize() != this.SIZE) {
            throw new IllegalArgumentException("Cannot add two vectors that do not contain the same number of elements.");
        }

        // Calculate the Vector result.
        double[] resultVectorValue = new double[this.SIZE];
        for (int i = 0; i < this.SIZE; i++) {
            resultVectorValue[i] = this.VALUES[i] - otherVector.getValueAt(i);
        }

        // Return the result as a new Vector.
        return new Vector(resultVectorValue);
    }

    /**
     * Divides all of this Vector's values by the given divisor and returns the result as a new Vector.
     *
     * @param divisor The value that each of this Vector's values will be multiplied by.
     * @return A new Vector containing the result of the Vector multiplication.
     */
    public Vector divideBy(double divisor) {

        // Divide each of this Vector's values by the given divisor.
        double[] resultVectorValue = new double[this.SIZE];
        for (int i = 0; i < this.SIZE; i++) {
            resultVectorValue[i] = this.VALUES[i] / divisor;
        }

        // Return the result as a new Vector.
        return new Vector(resultVectorValue);
    }

    /**
     * Multiplies all of this Vector's values by the given multiplier and returns the result as a new
     * Vector.
     *
     * @param multiplier The value that each of this Vector's values will be multiplied by.
     * @return A new Vector containing the values of this Vector multiplied by the given value.
     */
    public Vector multiplyBy(double multiplier) {

        // Multiply each of this Vector's values by the given multiplier.
        double[] resultVectorValue = new double[this.SIZE];
        for (int i = 0; i < this.SIZE; i++) {
            resultVectorValue[i] = this.VALUES[i] * multiplier;
        }

        // Return the result as a new Vector.
        return new Vector(resultVectorValue);
    }

    /**
     * Multiplies this Vector by the provided Vector and returns the result as a new Vector.
     * The given Vector must have the same number of elements as this Vector.
     *
     * @param otherVector The Vector who's elements will be multiplied by this Vector's values.
     * @return A new Vector containing the result of the Vector multiplication.
     * @throws IllegalArgumentException If the provided vector doesn't contain the same number of elements as this vector.
     */
    public Vector multiply(Vector otherVector) {

        // Make sure that both Vectors contain the same number of values.
        if (otherVector.getSize() != this.SIZE) {
            throw new IllegalArgumentException("Cannot multiply two vectors that do not contain the same number of elements.");
        }

        // Calculate the cross product of the two vectors.
        double[] resultVectorValue = new double[this.SIZE];
        for (int i = 0; i < this.SIZE; i++) {
            resultVectorValue[i] = this.VALUES[i] * otherVector.getValueAt(i);
        }

        // Return the result as a new Vector.
        return new Vector(resultVectorValue);
    }

    /**
     * Calculates the dot-product between this Vector and the provided Vector.
     * The given Vector must have the same number of elements as this Vector.
     *
     * @param otherVector The vector to calculate the dot-product between.
     * @return The dot product of the two vectors.
     * @throws IllegalArgumentException If this vector and the provided Vector do not have
     *                                  the same number of elements.
     */
    public double calculateDotProduct(Vector otherVector) {

        // Make sure the two vectors are the same size.
        if (this.getSize() != otherVector.getSize()) {
            throw new IllegalArgumentException("Cannot compute the dot product between two Vectors " +
                    "with different quantities of elements.");
        }

        // Calculate and return the dot product between the two vectors.
        return IntStream.range(0, this.getSize())
                .parallel()
                .mapToDouble(i -> this.VALUES[i] * otherVector.getValueAt(i))
                .sum();
    }

    /**
     * Projects this vector onto the given Vector.
     * The given Vector must have the same number of elements as this Vector.
     *
     * @param otherVector The Vector that this Vector will be projected onto.
     * @return This Vector projected onto the given Vector, as a new Vector object.
     * @throws IllegalArgumentException If the provided Vector doesn't have the same number of elements
     *                                  as this Vector.
     */
    public Vector projectOnto(Vector otherVector) {

        // Make sure that both Vectors have the same number of elements.
        if (otherVector.getSize() != this.SIZE) {
            throw new IllegalArgumentException("Cannot project Vector onto another Vector with a different number of elements.");
        }

        // Calculate necessary values to project this Vector onto the provided vector.
        double dotProduct = this.calculateDotProduct(otherVector);
        double otherVectorMagnitude = otherVector.getMagnitude();

        // Calculate the value the other Vector has to be multiplied by in order to get the
        // projected Vector.
        double scalar = dotProduct / Math.pow(otherVectorMagnitude, 2);

        // Return the result, as a Vector.
        return otherVector.multiplyBy(scalar);
    }

    /**
     * Calculates the angle between this Vector and the provided vector.
     * The given Vector must have the same number of elements as this Vector.
     *
     * @param otherVector The Vector that the angle will be calculated between.
     * @return The angle between this Vector and the provided Vector, in radians.
     * @throws IllegalArgumentException If the provided Vector doesn't have the same number of elements as this Vector.
     */
    public double getAngleBetween(Vector otherVector) {

        // Make sure that the vectors have the same number of elements.
        if (this.SIZE != otherVector.getSize()) {
            throw new IllegalArgumentException("Cannot calculate angle between two vectors with different numbers of elements.");
        }

        // Calculate necessary values to compute the angle between the two vectors.
        double dotProduct = this.calculateDotProduct(otherVector);
        double magnitudeSum = this.getMagnitude() * otherVector.getMagnitude();

        // Calculate and return the angle between the two vectors, in radians.
        return Math.acos(dotProduct / magnitudeSum);
    }

    /**
     * Returns this Vector's unit vector, as a new Vector object.
     *
     * @return This Vector's unit vector, as a new Vector object.
     */
    public Vector getUnitVector() {

        // Make sure that magnitude is calculated.
        if (!isMagnitudeCalculated) {
            this.magnitude = calculateMagnitude();
            this.isMagnitudeCalculated = true;
        }

        // This method returns a new Vector with all of its values divided by the given value.
        // Since we will be dividing all values by the magnitude, this method is sufficient to
        // calculate the unit vector.
        return this.divideBy(this.magnitude);
    }

    /**
     * Returns the magnitude of this vector.
     *
     * @return The magnitude of this vector.
     */
    public double getMagnitude() {

        // Check whether or not magnitude has already been calculated.
        // If it hasn't been calculated, then calculate it.
        if (!isMagnitudeCalculated) {
            this.magnitude = calculateMagnitude();
            this.isMagnitudeCalculated = true;
        }

        // Return this vector's magnitude.
        return this.magnitude;
    }

    /**
     * Calculates this Vector's magnitude.
     *
     * @return This Vector's magnitude.
     */
    private double calculateMagnitude() {

        // Calculate the sum of all elements squared, since the formula for magnitude is √x² + y² ...
        double sum = 0;
        for (double value : this.VALUES) {
            sum += (value * value);
        }

        // Return the square root of the sum to complete the calculation.
        return Math.sqrt(sum);
    }

    /**
     * Returns the number of values in this vector.
     *
     * @return The number of elements in this Vector.
     */
    public int getSize() {
        return this.SIZE;
    }

    /**
     * Sets the value at the specified index in the Vector to the given value.
     * The index must be valid and correspond to a pre-filled position in the Vector.
     *
     * @param index The index of the element you wish to replace.
     * @param value The new value to be assigned at the specified index.
     * @throws IndexOutOfBoundsException If the given index is out of bounds (greater than or equal to the number of elements in this Vector).
     */
    public void setValueAt(int index, double value) {

        // Ensure there is a value at the specified index.
        if (index >= this.SIZE || index < 0) {
            throw new IndexOutOfBoundsException("Cannot set element at position: " + index +
                    " - Invalid position.");
        }

        // Set the value at the specified index to the given value.
        this.VALUES[index] = value;

        // Magnitude needs to be recalculated, since the values have changed.
        this.isMagnitudeCalculated = false;
    }

    /**
     * Returns the value at the specified index.
     *
     * @param index The index of the value that will be returned.
     * @return The value at the specified index.
     * @throws IndexOutOfBoundsException If no value exists at the specified index,
     */
    public double getValueAt(int index) {

        // Make sure the element exists.
        if (index >= this.SIZE || index < 0) {
            throw new IndexOutOfBoundsException("Cannot obtain element at index " + index +
                    " in vector that only contains " + this.SIZE + " values.");
        }

        // Returns the value at the specified index.
        return this.VALUES[index];
    }
}