package org.firstinspires.ftc.teamcode.Utility.Storage;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.Utility.Constants.FileManagementConstants;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;
public class FileEx {

    public static final String directoryName = "FIRST";
    private final String filePath;
    private HashMap<String, String> fileData;


    /**
     * This constructor helps set up the class so that it is able to save and load data tp and from
     * the specified file.
     * @param fileName The name of the file you want to interact with.
     */
    public FileEx(String fileName) {

        // Create a file object for the directory.
        File directory = new File(FileManagementConstants.DIRECTORY_PATH);
        directory.mkdir();

        // Set the variable filePath equal to the path of the file.
        filePath = FileManagementConstants.DIRECTORY_PATH + "/" + fileName;

        // Get all of the data from the file. Then, if there isn't any data create a new hashmap.
        fileData = getAllData();
        if (fileData == null) {
            fileData = new HashMap<String, String>();
        }
    }

    /**
     * This method returns an HashMap containing all of the data stored in the file.
     *
     * @return Returns an HashMap containing all of the file's data. (If there is data to be found.)
     */
    public HashMap<String, String> getAllData() {

        // Extract the HashMap from the file.
        try (FileInputStream fileInputStream = new FileInputStream(filePath);
             ObjectInputStream objectInputStream = new ObjectInputStream(fileInputStream)) {

            // Reading the HashMap from the file.
            return (HashMap<String, String>) objectInputStream.readObject();

            // Tell the user that an error has occurred.
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * This method saves all data stored in the HashMap to a file.
     */
    public void saveData() {

        // Save the fileData HashMap to the file.
        try (FileOutputStream fileOutputStream = new FileOutputStream(filePath);
             ObjectOutputStream objectOutputStream = new ObjectOutputStream(fileOutputStream)) {

            // Writing the HashMap to the file
            objectOutputStream.writeObject(fileData);

            // Tell the user that an error has occurred.
        } catch (IOException error) {
            error.printStackTrace();
        }
    }

    /**
     * Saves a value to the file under the provided key (name).
     *
     * @param valueName The name you want to store the value under.
     * @param value The value you want to store.
     */
    public void addData(String valueName, Object value) {

        // Convert the provided value to a string then add it to the HashMap.
        fileData.put(valueName, String.valueOf(value));

        // Save the data to the file.
        saveData();
    }

    /**
     * This method gets a value from the file with the provided name.
     *
     * @param valueName The name of the value you want to get.
     * @return Returns the value (String) of the named variable.
     */
    public String getValue(String valueName) {
        return fileData.get(valueName);
    }

    /**
     * This method gets a value from the file with the provided name.
     *
     * @param valueName The name of the value you want to get.
     * @param conversionType The type you want the value returned to be.
     * @return Returns the value (String) of the named variable of type conversionType.
     */
    public <T> T getValue(String valueName, Class<T> conversionType) {

        // Get the value associated with the provided name. Then, if the value is null set it to 0.
        String value = fileData.get(valueName);
        if (value == null) {
            return null;
        }

        // Logic to convert the value to the desired data type.
        Object result;
        switch (conversionType.getSimpleName()) {
            case "Integer":
                result = Integer.parseInt(value);
                break;
            case "Long":
                result = Long.parseLong(value);
                break;
            case "Double":
                result = Double.parseDouble(value);
                break;
            case "Float":
                result = Float.parseFloat(value);
                break;
            case "Boolean":
                result = Boolean.parseBoolean(value);
                break;
            case "String":
                result = value;
                break;
            default:
                throw new IllegalArgumentException("Unsupported data type: " + conversionType.getName());
        }

        // Return the result
        return conversionType.cast(result);
    }
}