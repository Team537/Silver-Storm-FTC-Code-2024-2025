package org.firstinspires.ftc.teamcode.Utility;

import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public final class Constants {

    public static final class CommandConstants {
        public static final String COMMAND_SEPARATION_CHARACTER = "/";
    }

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

        // Camera Specs
        public static final class LogitechC270Constants {
            // TODO: Get camera parameters!
        }

        public static final class LogitechBrio100Constants {
            public static final int STREAM_WIDTH_PIXELS = 640;
            public static final int STREAM_HEIGHT_PIXELS = 480;

            // Calibrated camera matrix values.
            // NOTE: These values may be inaccurate due to potential issues with CalibCalibration.
            public static Mat CAMERA_MATRIX = new Mat(3, 3, CvType.CV_64FC1); // CvType.CV_64FC1 is used to store doubles, providing the necessary precision for calibration data.

            public static Mat CAMERA_ROTATION_MATRIX = new Mat(3, 3, CvType.CV_64FC1);
            public static Mat CAMERA_TRANSLATION_MATRIX = new Mat(3, 1, CvType.CV_64FC1);

            private static double[] cameraMatrixValues = new double[]{
                    876.7542742312495, 0, 328.9249436771823,  // First row of the camera matrix
                    0, 878.198963669983, 210.28863286011176,   // Second row of the camera matrix
                    0, 0, 1                                    // Third row of the camera matrix (homogeneous coordinate)
            };

            private static double[] cameraRotationMatrixValues = new double[]{
                    1, 0, 0,   // First row of the camera rotation matrix
                    0, 1, 0,   // Second row of the camera rotation matrix
                    0, 0, 1    // Third row of the camera rotation matrix
            };

            private static double[] cameraTranslationMatrixValues = new double[]{
                    0,   // First row of the camera translation matrix
                    0,   // Second row of the camera translation matrix
                    0, // Third row of the camera translation matrix
            };

            // Static block used to populate the camera matrix with values.
            // The put() method can only be called within a static block for static fields.
            static {
                CAMERA_MATRIX.put(0, 0, cameraMatrixValues);
                CAMERA_ROTATION_MATRIX.put(0, 0, cameraRotationMatrixValues);
                CAMERA_TRANSLATION_MATRIX.put(0, 0, cameraTranslationMatrixValues);
            }

            // Distortion coefficients for the camera.
            // These coefficients account for lens distortion during image capture.
            public static final Mat DISTORTION_COEFFICIENTS = new MatOfDouble(
                    0.20235229291431436,
                    -0.7581107475435005,
                    -0.0005196745875307539,
                    -0.0020279369814570638,
                    0.26706090549820094
            );
        }
    }
}
