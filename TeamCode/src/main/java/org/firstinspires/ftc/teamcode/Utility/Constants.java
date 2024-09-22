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
}
