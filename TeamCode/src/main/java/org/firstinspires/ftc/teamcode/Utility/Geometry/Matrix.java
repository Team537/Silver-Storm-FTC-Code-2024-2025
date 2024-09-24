package org.firstinspires.ftc.teamcode.Utility.Geometry;

import java.util.stream.IntStream;

public class Matrix {

    double[][] matrixValues;
    private final int COLUMNS;
    private final int ROWS;

    /**
     * Creates an empty matrix with the specified number of columns and rows.
     *
     * @param columns The number of columns this matrix will contain.
     * @param rows The number of rows this matrix will contain.
     */
    public Matrix(int columns, int rows) {

        // Create a new matrix with the specified values.
        this.matrixValues = new double[columns][rows];

        // Save the number of columns and rows that this matrix contains.
        this.COLUMNS = columns;
        this.ROWS = rows;
    }

    /**
     * Attempts to create a matrix with the given 2D array of matrix values.
     *
     * @param matrixValues The values that will be contained within this Matrix.
     * @throws IllegalArgumentException If the matrix doesn't contain the same number of elements in each row.
     */
    public Matrix(double[][] matrixValues) {

        // Make sure the provided matrix is a valid matrix.
        boolean validMatrix = isValidMatrix(matrixValues);
        if (!validMatrix) {
            throw new IllegalArgumentException("Matrix cannot be created without the same number of elements in each row.");
        }

        // Set this matrix's values to the given 2D array, since we know it is a valid matrix.
        this.matrixValues = matrixValues;

        // Save the number of columns and rows that this matrix contains.
        this.ROWS = matrixValues.length;
        this.COLUMNS = matrixValues[0].length;
    }

    /**
     * Checks if the given 2D array represents a valid matrix, meaning all rows must have the same number of elements.
     *
     * @param matrixValues A 2D array representing potential matrix values.
     * @return {@code true} if all rows have the same number of elements, {@code false} otherwise.
     * @throws IllegalArgumentException if the matrixValues array is null or has no rows.
     */
    private boolean isValidMatrix(double[][] matrixValues) {

        // Keep track of whether or not each row has the same number of elements.
        boolean validMatrix = true;

        // Loop through each row and make sure each row contains the same number of elements.
        // < length - 1 is used here since we are already checking the last row when rowNumber = length - 2.
        for (int rowNumber = 0; rowNumber < (matrixValues.length -1); rowNumber++) {

            // If there are more / less elements in this array
            if (matrixValues[rowNumber].length != matrixValues[rowNumber + 1].length) {
                validMatrix = false;
                break;
            }
        }

        // Return whether or not the matrix is a valid matrix.
        return validMatrix;
    }

    /**
     * Adds the values from the provided Matrix to this matrix and returns the resulting Matrix.
     *
     * @param otherMatrix The Matrix whose values will be added to this Matrix.
     * @return A new Matrix representing the sum of this Matrix and the provided Matrix.
     * @throws IllegalArgumentException If the provided Matrix does not have the same number of
     *                                  rows and columns as this Matrix.
     */
    public Matrix addMatrix(Matrix otherMatrix) {

        // Ensure that both matrices have the same number of rows and columns.
        if (this.ROWS != otherMatrix.getRows() || this.COLUMNS != otherMatrix.getColumns()) {
            throw new IllegalArgumentException("Cannot add matrices that do not have the same number of rows and columns");
        }

        // Create a 2D array to store the values of the resulting Matrix.
        double[][] resultMatrixValues = new double[this.ROWS][this.COLUMNS];

        // Loop through each row and column to calculate the sum.
        for (int i = 0; i < this.ROWS * this.COLUMNS; i++) {
            int row = i / this.COLUMNS; // Calculate the row index.
            int column = i % this.COLUMNS; // Calculate the column index.

            // Calculate the sum of this Matrix's value and the other Matrix's value.
            resultMatrixValues[row][column] = this.matrixValues[row][column] +
                    otherMatrix.getValueAt(row, column);
        }

        // Return a new Matrix containing the resulting values.
        return new Matrix(resultMatrixValues);
    }

    /**
     * Subtracts the values of the provided Matrix from this Matrix and returns the resulting Matrix.
     *
     * @param otherMatrix The Matrix whose values will be subtracted from this Matrix's values.
     * @return A new Matrix containing the values of this Matrix subtracted by the values of the
     *         provided Matrix.
     * @throws IllegalArgumentException If the provided Matrix does not have the same number of
     *                                  rows and columns as this Matrix.
     */
    public Matrix subtractMatrix(Matrix otherMatrix) {

        // Ensure that both matrices have the same number of rows and columns.
        if (this.ROWS != otherMatrix.getRows() || this.COLUMNS != otherMatrix.getColumns()) {
            throw new IllegalArgumentException("Cannot add matrices that do not have the same number of rows and columns");
        }

        // Create a 2D array to store the values of the resulting matrix.
        double[][] resultMatrixValues = new double[this.ROWS][this.COLUMNS];

        // Loop through each row and column and calculate a Matrix containing this Matrix's values
        // subtracted by the prvodied Matrix's values.
        for (int i = 0; i < this.ROWS * this.COLUMNS; i++) {
            int row = i / this.COLUMNS; // Calculate the row index.
            int column = i % this.COLUMNS; // Calculate the column index.

            // Subtracts the value of the other Matrix from this Matrix's value at the calculated
            // row and column.
            resultMatrixValues[row][column] = this.matrixValues[row][column] -
                    otherMatrix.getValueAt(row, column);
        }

        // Return a new Matrix containing the resulting values.
        return new Matrix(resultMatrixValues);
    }

    /**
     * Multiplies every value of this Matrix by the provided value and returns the result as a new Matrix.
     *
     * @param multiplier The value that each Matrix element will be multiplied by.
     * @return A new Matrix representing the product of this matrix and the given multiplier.
     */
    public Matrix multiplyBy(double multiplier) {

        // Create a 2D array to store the values of the resulting Matrix.
        double[][] resultMatrixValues = new double[this.ROWS][this.COLUMNS];

        // Loop through each row and column and multiply each value by the provided multiplier.
        for (int i = 0; i < this.ROWS * this.COLUMNS; i++) {
            int row = i / this.COLUMNS; // Calculate the row index.
            int column = i % this.COLUMNS; // Calculate the column index.

            // Multiply the value at the calculated column and row by the given multiplier.
            resultMatrixValues[row][column] = this.matrixValues[row][column] * multiplier;
        }

        // Return a new Matrix containing the resulting values.
        return new Matrix(resultMatrixValues);
    }

    /**
     * Divides every value of this Matrix by the provided divisor and returns the result as a new Matrix.
     *
     * @param divisor The value that each Matrix element will be divided by.
     * @return A new Matrix representing the product of this matrix and the given multiplier.
     */
    public Matrix divideBy(double divisor) {

        // Create a 2D array to store the values of the resulting Matrix.
        double[][] resultMatrixValues = new double[this.ROWS][this.COLUMNS];

        // Loop through each row and column and divide each value by the provided divisor.
        for (int i = 0; i < this.ROWS * this.COLUMNS; i++) {
            int row = i / this.COLUMNS; // Calculate the row index.
            int column = i % this.COLUMNS; // Calculate the column index.

            // Divide the value at the calculated column and row by the given divisor.
            resultMatrixValues[row][column] = this.matrixValues[row][column] / divisor;
        }

        // Return a new Matrix containing the resulting values.
        return new Matrix(resultMatrixValues);
    }

    /**
     * Multiples this Matrix by the provided Matrix and returns the result as a new Matrix.
     *
     * @param otherMatrix The Matrix who's values will be multiplied with this Matrix's values.
     * @return A matrix with the values of this Matrix multiplied by the provided Matrix.
     * @throws IllegalArgumentException If this Matrix's number of columns isn't equal to the provided Matrix's
     *                                  number of row.
     */
    public Matrix multiply(Matrix otherMatrix) {

        // Make sure we can multiply the two matrices together.
        // If we cannot preform the multiplication operation, output an error to inform the programmer / user.
        if (this.COLUMNS != otherMatrix.getRows()) {
            throw new IllegalArgumentException("Cannot multiply matrix with " + otherMatrix.getRows() +
                    " rows with a matrix with " + this.COLUMNS + "columns.");
        }

        // Initializes the result matrix with the proper dimensions (this.ROWS by otherMatrix.COLUMNS)
        double[][] resultMatrixValues = new double[this.ROWS][otherMatrix.getColumns()];

        // Preform matrix multiplication.
        for (int rowNumber = 0; rowNumber < this.ROWS; rowNumber++) {
            for (int columnNumber = 0; columnNumber < otherMatrix.getColumns(); columnNumber++) {
                resultMatrixValues[rowNumber][columnNumber] = calculateDotProduct(this.matrixValues[rowNumber], otherMatrix.getColumn(columnNumber));
            }
        }

        // Return a new Matrix containing the resulting values.
        return new Matrix(resultMatrixValues);
    }

    /**
     * Calculates the dot-product between two vectors.
     * Both vectors must be the same length.
     *
     * @param vectorA The first input vector.
     * @param vectorB The second input vector.
     * @return The dot-product of the two vectors.
     * @throws IllegalArgumentException If the vectors are null or aren't the same length.
     */
    private double calculateDotProduct(double[] vectorA, double[] vectorB) {

        // Make sure the input vectors aren't null and are of the same length.
        if (vectorA == null || vectorB == null || vectorA.length != vectorB.length) {
            throw new IllegalArgumentException("Vectors must be non-null and of the same length.");
        }

        // Calculate the dot product of the two vectors.
        return IntStream.range(0, vectorA.length)
                .parallel() // Run processes across multiple threads to calculate the result quicker.
                .mapToDouble(i -> vectorA[i] * vectorB[i]) // Preform a method on each element of the stream range.
                .sum(); // Return the sum of all elements in the stream.
    }

    /**
     * Returns the number of rows present within this Matrix.
     *
     * @return The number of rows present within this Matrix.
     */
    public int getRows() {
        return this.ROWS;
    }

    /**
     * Returns the number of columns present within this Matrix.
     *
     * @return The number of columns present within this Matrix.
     */
    public int getColumns() {
        return this.COLUMNS;
    }

    /**
     * Returns an array containing all values within this Matrix's rowNum row.
     *
     * @param rowNum The number of the row you wish to access.
     * @return An array containing all values within this matrix's rowNum row.
     * @throws IndexOutOfBoundsException If the requested row does not exist.
     */
    public double[] getRow(int rowNum) {

        // Make sure the specified row exists.
        if (rowNum >= this.ROWS) {
            throw new IndexOutOfBoundsException("Specified row of matrix out of bounds.");
        }

        // Return an array containing all values in the specified row.
        return this.matrixValues[rowNum];
    }

    /**
     * Returns an array containing all values within this Matrix's columnNum column.
     *
     * @param columnNum The number of the column you wish to access.
     * @return An array containing all values within this Matrix's columnNum column.
     * @throws IndexOutOfBoundsException If the requested column does not exist.
     */
    public double[] getColumn(int columnNum) {

        // Make sure the specified column exists.
        if (columnNum >= this.COLUMNS) {
            throw new IndexOutOfBoundsException("Specified column of matrix out of bounds.");
        }

        // Loop through all rows in this Matrix and grab the element located in the specified column.
        double[] columValues = new double[this.ROWS];
        for (int row = 0; row < this.ROWS; row++) {
            columValues[row] = this.matrixValues[row][columnNum];
        }

        // Return an array containing all elements within the specified column.
        return columValues;
    }

    /**
     * Returns the value of this Matrix in the specified row and column.
     *
     * @param row The row the value is in.
     * @param column The column the value is in.
     * @return The value of this Matrix in the specified row and column.
     * @throws IndexOutOfBoundsException If this Matrix does not have the specified number of columns
     *                                   and/or rows.
     */
    public double getValueAt(int row, int column) {

        // Make sure that this Matrix has the specified number of rows and columns.
        if (row >= this.ROWS || column >= this.COLUMNS) {
            throw new IndexOutOfBoundsException("Requested element out of range, matrix " +
                    "doesn't have the specified number of rows and columns.");
        }

        // Return the value at the specified row and column.
        return this.matrixValues[row][column];
    }

    /**
     * Converts this matrix into a string. The resulting output is similar to standard Matrix notation.
     *
     * @return This matrix, as a string.
     */
    @Override
    public String toString() {

        // Loop through all of the rows and columns of this Matrix and format the string in a
        // similar fashion to standard Matrix notation.
        StringBuilder stringBuilder = new StringBuilder();
        for (int rowNumber = 0; rowNumber < this.ROWS; rowNumber++) {

            // Add a vertical line at the start of each row.
            stringBuilder.append('|');

            // Loop through all the elements in a row and add it to the output string.
            for (int columnNumber = 0; columnNumber < this.COLUMNS; columnNumber++) {

                // Add the value located in the specific row and column to the output string.
                stringBuilder.append(this.matrixValues[rowNumber][columnNumber]);

                // If the value isn't at the end of a row, add a comma after it.
                if (columnNumber != this.COLUMNS - 1) {
                    stringBuilder.append(", ");
                }
            }

            // Add a vertical line at the end of each row.
            stringBuilder.append("|\n");
        }

        // Return the Matrix in string form.
        return stringBuilder.toString();
    }
}