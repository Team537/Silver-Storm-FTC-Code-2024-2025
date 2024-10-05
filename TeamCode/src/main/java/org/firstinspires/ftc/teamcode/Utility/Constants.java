package org.firstinspires.ftc.teamcode.Utility;

import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

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

        // Camera Specs
        public static final class LogitechC270Constants {
            public static final int STREAM_WIDTH_PIXELS = 320;
            public static final int STREAM_HEIGHT_PIXELS = 240;
            public static final int FOCAL_LENGTH = 720;
        }

        public static final class LogitechBrio100Constants {
            public static final int STREAM_WIDTH_PIXELS = 640;
            public static final int STREAM_HEIGHT_PIXELS = 480;

            // Calibrated camera matrix values.
            // NOTE: These values may be inaccurate due to potential issues with CalibCalibration.
            public static Mat CAMERA_MATRIX = new Mat(3, 3, CvType.CV_64FC1); // CvType.CV_64FC1 is used to store doubles, providing the necessary precision for calibration data.

            private static double[] cameraMatrixValues = new double[]{
                    881.1893151638046, 0, 292.5860656237113,   // First row of the camera matrix
                    0, 886.2559682260007, 220.5268346349555,   // Second row of the camera matrix
                    0, 0, 1                                    // Third row of the camera matrix (homogeneous coordinate)
            };

            // Static block used to populate the camera matrix with values.
            // The put() method can only be called within a static block for static fields.
            static {
                CAMERA_MATRIX.put(0, 0, cameraMatrixValues);
            }

            // Distortion coefficients for the camera.
            // These coefficients account for lens distortion during image capture.
            public static final Mat DISTORTION_COEFFICIENTS = new MatOfDouble(
                    -0.1952774644915585,   // Radial distortion coefficient (k1)
                    0.5880599936388096,    // Radial distortion coefficient (k2)
                    0.0008952909724109531, // Tangential distortion coefficient (p1)
                    -0.0020515149003748276,// Tangential distortion coefficient (p2)
                    -0.6013858183664286    // Radial distortion coefficient (k3)
            );
        }
    }
}
