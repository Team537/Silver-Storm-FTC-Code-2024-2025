package org.firstinspires.ftc.teamcode.Utility;

import android.os.Environment;

public final class Constants {

    public static final class DrivetrainConstants {

        // Store the maximum possible speeds that the drive motors can turn at.
        public static final double MAX_MOTOR_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 36.6519142919;
        public static final double MAX_MOTOR_POWER = 1;

        // The robot's x velocity will be multiplied by this value. Doing this helps counteract
        // imperfect straifing.
        public static final double STRAIF_OFFSET_MULTIPLIER = 1.1;
    }

    public static final class FileManagementConstants {

        // The name of the folder FIRST allows you to save files to.
        public static final String DIRECTORY_NAME = "FIRST";

        // Get the path to the directory where all external data will be stored.
        public static String DIRECTORY_PATH = Environment.getExternalStorageDirectory().getPath() + "/" + DIRECTORY_NAME;
    }

    public static final class VisionConstants {

        // Camera Names
        public static final String TEST_CAMERA_NAME = "Webcam";
        // Camera Dimensions
        public static final int CAMERA_WIDTH_PIXELS = 320; // 640
        public static final int CAMERA_HEIGHT_PIXELS = 240; // 480
        public static final int FOCAL_LENGTH_PIXELS = 720; // Sourced from amazon; may not be perfectly accurate.
    }
}
