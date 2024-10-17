package org.firstinspires.ftc.teamcode.Utility;

import android.os.Environment;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

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

    public static final class IMUConstants {

        public static final double[] STARTING_STATE_MATRIX_VALUES = new double[] {
                0, 0
        };

        // These values determine how the IMU's current state will change given no inputs.
        public static final double[] STATE_TRANSITION_MATRIX_VALUES = new double[] {
                1, 0, // Second element will be the change in time.
                0, 1 // Friction is lame.
        };

        // We know our starting orientation, so our starting estimation covariance matrix values are low.
        public static final double[] STARTING_ESTIMATION_COVARIANCE_MATRIX_VALUES = new double[] {
                0, 0,
                0, 0
        };

        // Values must be tuned! - How much do we trust our perfect world estimation at the start of the match?
        public static final double[] PROCESS_NOISE_COVARIANCE_MATRIX_VALUES = new double[] {
                0.01, 0,
                0, 0.005
        };

        // How the state matrix related to the measured values.
        public static final double[] OBSERVATION_MATRIX_VALUES = new double[] {
                1, 0,
                0, 1
        };

        // Sourced from https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        public static final double[] SENSOR_NOISE_MATRIX_VALUES = new double[] {
                0.0006981317, 0,
                0, 0.1 // Technically a 1x1 matrix, but we cannot add a scalar to 2x2 matrices, so we need to represent it as a 2x2 matrix.
        };


    }
}
