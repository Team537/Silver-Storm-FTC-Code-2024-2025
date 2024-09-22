package org.firstinspires.ftc.teamcode.Utility.Storage;

import org.firstinspires.ftc.teamcode.Utility.Constants.FileManagementConstants;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class DataLogger {

    // Settings
    private final String BASE_FILE_NAME = "Data Log-";
    private final String FILE_EXTENSION = ".csv";
    private final char SEPARATOR_CHARACTER = ',';

    // Storage
    private File file;
    private BufferedWriter fileWriter;

    /**
     * Constructs a DataLogger instance with specified data headers.
     *
     * @param dataHeaders A list of strings representing the names of the values
     *                    that the DataLogger will log. The order of these headers
     *                    is significant; when logging data, the corresponding
     *                    values must be provided in the same order as the headers.
     *                    For example, if the headers are "Time, XPosition, YPosition",
     *                    the logged values must also be in the order of
     *                    [time, xPosition, yPosition].
     */
    public DataLogger(List<String> dataHeaders) {

        // Get the current time.
        double timeMs = System.currentTimeMillis();

        // Create a new log file, with the name of the current time in ms in order to make it unique and identifiable.
        this.file = new File(FileManagementConstants.DIRECTORY_PATH + "/" + BASE_FILE_NAME + timeMs + FILE_EXTENSION);

        // Make it possible to write data to the file.
        this.openLogFileWriter(dataHeaders);
    }

    /**
     * Constructs a DataLogger instance with specified data headers and specified file name.
     *
     * @param fileName The name you of the file you wish for this DataLogger to use.
     * @param dataHeaders A list of strings representing the names of the values
     *                    that the DataLogger will log. The order of these headers
     *                    is significant; when logging data, the corresponding
     *                    values must be provided in the same order as the headers.
     *                    For example, if the headers are "Time, XPosition, YPosition",
     *                    the logged values must also be in the order of
     *                    [time, xPosition, yPosition].
     */
    public DataLogger(String fileName, List<String> dataHeaders) {

        // Create a new log file, with the name of the current time in ms in order to make it unique and identifiable.
        this.file = new File(FileManagementConstants.DIRECTORY_PATH + "/" + fileName + FILE_EXTENSION);

        // Make it possible to write data to the file.
        this.openLogFileWriter(dataHeaders);
    }

    /**
     * Create a fileWriter instance so that data may be logged, and adds the specified headers to
     * the top of the file.
     *
     * @param dataHeaders The list of headers this DataLogger will use.
     */
    private void openLogFileWriter(List<String> dataHeaders) {
        try {

            // Create a buffer writer to save data.
            // This is used instead of the regular file writer, as it is more efficient at writing data.
            this.fileWriter = new BufferedWriter(new FileWriter(file, true));

            // Setup the headers for the CSV file. This will make it clear what logged values are.
            this.logData(dataHeaders);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Saves the provided list of data to the file in a CSV format. This allows it to be imported into
     * other software so that it can be more easily viewed and manipulated.
     *
     * @param data A list of data that wul be logged to the file.
     * @param <T> A generic type representing the type of the items in the list of data.
     */
    public <T> void logData(List<T> data) {

        // Attempts to save the given data to a file.
        try {

            // Convert the provided data to a CSV format so that it can read and viewed by other programs.
            String formattedData = this.formatData(data);

            // Attempts to write the formatted data to the file.
            this.fileWriter.write(formattedData);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Converts all the data in the unformattedData list into a single string that can be saved as a
     * row in a CSV file.
     *
     * @param unformattedData A list of the data that needs to be formatted.
     * @return The formatted data, as a single string.
     * @param <T> A generic type representing the type of the items in the list of unformatted data.
     */
    private <T> String formatData(List<T> unformattedData) {

        // Store the formatted data as a StringBuilder. This offers several performance enhancements,
        // as it does not create a new object every time it is altered.
        StringBuilder formattedData = new StringBuilder();

        // Loop through all of the data.
        // Convert each data entry into a string and add it, alongside the separation character to
        // the formatted data.
        for (T data : unformattedData) {
            formattedData.append(data).append(SEPARATOR_CHARACTER);
        }

        // If the formatted data has a length greater than -, then remove the last character added,
        // since we don't need a separation character at the end of the CSV file.
        if (formattedData.length() > 0) {
            formattedData.setCharAt(formattedData.length() - 1, ' ');
        }

        // Add an escape character to the end of the line so that the file writer moves onto the next line.
        formattedData.append("\n");

        // Return the formatted data.
        return formattedData.toString();
    }

    /**
     * Closes this DataLogger's FileWriter. Prevents further data from being saved. This method must
     * be called in order for data to be saved.
     */
    public void close() {

        // Attempts to close the file writer.
        // If this fails, the data will not be saved.
        try {
            this.fileWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}