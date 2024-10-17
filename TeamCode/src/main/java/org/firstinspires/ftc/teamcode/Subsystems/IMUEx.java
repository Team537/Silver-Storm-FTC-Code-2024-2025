package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.Constants.IMUConstants;
import org.firstinspires.ftc.teamcode.Utility.Storage.DataLogger;
import org.firstinspires.ftc.teamcode.Utility.Time.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utility.Time.TimeUnit;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class IMUEx implements Subsystem {

    private IMU robotImu;
    private ElapsedTime timeSinceLastEstimation;
    private ElapsedTime runtime;
    private DataLogger dataLogger;

    // Storage
    private Telemetry telemetry;

    private Mat previousState = new Mat(2, 1, CvType.CV_64FC1);
    private Mat previousPredictedEstimateCovariance = new Mat(2, 2, CvType.CV_64FC1);

    private Mat stateTransitionMatrix = new Mat(2, 2, CvType.CV_64FC1);
    private Mat transposedStateTransitionMatrix = new Mat();
    private Mat processNoiseMatrix = new Mat(2, 2, CvType.CV_64FC1);
    private Mat observationMatrix = new Mat(2, 2, CvType.CV_64FC1);
    private Mat transposedObservationMatrix = new Mat();
    private Mat sensorNoiseMatrix = new Mat(2,2, CvType.CV_64FC1);
    private Mat identityMatrix = Mat.eye(2, 2, CvType.CV_64FC1);

    /**
     * Constructor for IMUEx. Initializes elapsed time tracking and data logging for IMU accuracy.
     */
    public IMUEx() {

        // Setup elapsed times.
        timeSinceLastEstimation = new ElapsedTime();
        runtime = new ElapsedTime();

        // Setup data logger.
        dataLogger = new DataLogger("IMU Accuracy Data", List.of("Time (s)", "Measured θ",
                "Measured ω", "Estimated θ", "Estimated ω","Optimal Estimation θ", "Optimal Estimation ω"));

        // Initialize Kalman filter matrices with constants defined in IMUConstants
        previousState.put(0, 0, IMUConstants.STARTING_STATE_MATRIX_VALUES);
        stateTransitionMatrix.put(0, 0, IMUConstants.STATE_TRANSITION_MATRIX_VALUES);
        previousPredictedEstimateCovariance.put(0, 0, IMUConstants.STARTING_ESTIMATION_COVARIANCE_MATRIX_VALUES);
        processNoiseMatrix.put(0, 0, IMUConstants.PROCESS_NOISE_COVARIANCE_MATRIX_VALUES);
        observationMatrix.put(0, 0, IMUConstants.OBSERVATION_MATRIX_VALUES);
        sensorNoiseMatrix.put(0, 0, IMUConstants.SENSOR_NOISE_MATRIX_VALUES);

        // Precompute transposed matrices for later use in matrix operations
        Core.transpose(stateTransitionMatrix, transposedStateTransitionMatrix);
        Core.transpose(observationMatrix, transposedObservationMatrix);
    }

    /**
     * Initializes the IMU and sets up its orientation based on the robot's layout.
     *
     * @param hardwareMap The hardware map from the robot configuration
     * @param telemetry   The telemetry object used for sending data to the driver station
     */
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Configure IMU with parameters, including its orientation on the robot
        IMU.Parameters imuSettings = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        // Initialize the IMU sensor and reset yaw (heading) to zero
        robotImu = hardwareMap.get(IMU.class, "imu");
        robotImu.initialize(imuSettings);
        robotImu.resetYaw();
    }

    /**
     * Runs the Kalman filter algorithm to estimate the current system state and updates telemetry.
     */
    public void performKalmanFiltering() {

        // Estimate state and covariance using Kalman filter
        Mat predictedState = predictState();
        Mat predictedCovariance = predictCovariance();
        Mat kalmanGain = calculateKalmanGain(predictedCovariance);
        Mat observedState = getMeasuredStateMatrix();
        Mat updatedState = calculateOptimalStateMatrix(predictedState, kalmanGain, observedState);
        Mat updatedCovariance = calculateEstimateCovarianceMatrix(kalmanGain, predictedCovariance);

        // Update telemetry with current states
        telemetry.addData("Measured θ", observedState.get(0, 0)[0]);
        telemetry.addData("Measured ω", observedState.get(1, 0)[0]);

        telemetry.addData("Perfect World Estimated θ", predictedState.get(0, 0)[0]);
        telemetry.addData("Perfect World Estimated ω", predictedState.get(1, 0)[0]);

        telemetry.addData("Estimated θ", updatedState .get(0, 0)[0]);
        telemetry.addData("Estimated ω", updatedState .get(1, 0)[0]);

        telemetry.update();

        // Log data to file
        dataLogger.logData(List.of(runtime.getElapsedTime(TimeUnit.SECOND), observedState.get(0, 0)[0],
                observedState.get(1, 0)[0], predictedState.get(0, 0)[0],
                predictedState.get(1, 0)[0], updatedState.get(0, 0)[0],
                updatedState.get(1, 0)[0]));

        // Update the previous state and covariance for the next iteration
        previousState = updatedState;
        previousPredictedEstimateCovariance = updatedCovariance;
    }

    public void save() {
        dataLogger.close();
    }

    /**
     * Estimates the current system state (θ and ω) using the state transition matrix.
     *
     * @return The estimated state matrix.
     */
    public Mat predictState() {

        // Create a new matrix to store the estimated positional values.
        Mat estimatedSystemState = new Mat(2, 1, CvType.CV_32FC1);

        // Time elapsed since the last state estimation
        double deltaTime = timeSinceLastEstimation.getElapsedTime(TimeUnit.SECOND);
        timeSinceLastEstimation.reset();

        // Update the state transition matrix with the time difference
        stateTransitionMatrix.put(0, 1, deltaTime);
        stateTransitionMatrix.put(1, 1, deltaTime * 0.5);

        // Compute the estimated system state
        Core.gemm(stateTransitionMatrix, previousState, 1, new Mat(), 0, estimatedSystemState);

        // TODO: Account for rotational change resulting from motor rotation.

        // Return the estimated system state.
        return estimatedSystemState;
    }

    /**
     * Predicts the covariance matrix for the estimated state, considering process noise and the
     * state transition matrix.
     *
     * @return The predicted estimate covariance matrix.
     */
    public Mat predictCovariance() {

        Mat predictedEstimateCovariance = new Mat(2, 2, CvType.CV_32FC1);

        Mat matrixSum = new Mat();
        Core.gemm(stateTransitionMatrix, previousPredictedEstimateCovariance, 1, new Mat(), 0, matrixSum);
        Core.gemm(matrixSum, transposedStateTransitionMatrix, 1, new Mat(), 0, matrixSum);

        Core.add(matrixSum, processNoiseMatrix, predictedEstimateCovariance);

        return predictedEstimateCovariance;
    }

    /**
     * Calculates the Kalman gain matrix, which determines how much weight to give to the sensor's
     * measured state versus the estimated state.
     *
     * @param predictedEstimateCovariance The predicted estimate covariance matrix.
     * @return The Kalman gain matrix.
     */
    public Mat calculateKalmanGain(Mat predictedEstimateCovariance) {

        Mat kalmanGain = new Mat();

        // Step 1: Calculate H^T * P^-
        Mat tempMatrix1 = new Mat();
        Core.gemm(predictedEstimateCovariance, transposedObservationMatrix, 1, new Mat(), 0, tempMatrix1);

        // Step 2: Calculate H * P^- * H^T + R
        Mat tempMatrix2  = new Mat();
        Core.gemm(observationMatrix, predictedEstimateCovariance, 1, new Mat(), 0, tempMatrix2 );
        Core.gemm(tempMatrix2 , transposedObservationMatrix, 1, new Mat(), 0, tempMatrix2 );
        Core.add(tempMatrix2 , sensorNoiseMatrix, tempMatrix2 ); // Add R (sensor noise)

        // Step 3: Invert (H * P^- * H^T + R)
        Mat invertedTempMatrix  = new Mat();
        Core.invert(tempMatrix2, invertedTempMatrix );

        // Step 4: Calculate K = P^- * H^T * (H * P^- * H^T + R)^-1
        Core.gemm(tempMatrix1, invertedTempMatrix , 1, new Mat(), 0, kalmanGain);

        return kalmanGain;
    }

    /**
     * Calculates the optimal state matrix using the estimated state, Kalman gain, and measured state.
     *
     * The optimal state matrix is computed by incorporating the Kalman gain and the difference between
     * the measured state and the predicted state based on the observation matrix.
     *
     * @param estimatedStateMatrix The estimated state matrix containing the predicted orientation and angular velocity.
     * @param kalmanGain          The Kalman gain matrix, which indicates the weighting of the measurement update.
     * @param measuredStateMatrix  The measured state matrix containing the current orientation and angular velocity.
     * @return The optimal state matrix after applying the Kalman filter update.
     */
    public Mat calculateOptimalStateMatrix(Mat estimatedStateMatrix, Mat kalmanGain, Mat measuredStateMatrix) {

        Mat optimalStateMatrix = new Mat();

        Mat tempMatrix1 = new Mat();
        Core.gemm(observationMatrix, estimatedStateMatrix, 1, new Mat(), 0, tempMatrix1);
        Core.subtract(measuredStateMatrix, tempMatrix1, tempMatrix1);
        Core.gemm(kalmanGain, tempMatrix1, 1, new Mat(), 0, tempMatrix1);

        Core.add(estimatedStateMatrix, tempMatrix1, optimalStateMatrix);

        return optimalStateMatrix;
    }

    /**
     * Calculates the updated estimate covariance matrix based on the Kalman gain and predicted covariance matrix.
     *
     * This method computes the estimate covariance matrix by applying the Kalman gain to the predicted covariance
     * matrix, reflecting the uncertainty in the state estimate after incorporating the measurement.
     *
     * @param kalmanGain The Kalman gain matrix, which determines how much the measurements should
     *                   influence the state estimate.
     * @param predictedCovarianceMatrix  The predicted estimate covariance matrix before the measurement update.
     * @return The updated estimate covariance matrix after the measurement update.
     */
    public Mat calculateEstimateCovarianceMatrix(Mat kalmanGain, Mat predictedCovarianceMatrix) {

        // Matrix to hold the updated estimate covariance
        Mat estimateCovarianceMatrix = new Mat();

        // Temporary matrix for intermediate calculations
        Mat tempMatrix = new Mat();

        // Step 1: Calculate K * H
        Core.gemm(kalmanGain, observationMatrix, 1, new Mat(), 0, tempMatrix);

        // Step 2: Compute (I - K * H)
        Core.subtract(identityMatrix, tempMatrix, tempMatrix);

        // Step 3: Update the estimate covariance matrix
        Core.gemm(tempMatrix, predictedCovarianceMatrix, 1, new Mat(), 0, estimateCovarianceMatrix);

        // Return the updated estimate covariance matrix
        return estimateCovarianceMatrix;
    }

    /**
     * Retrieves the measured state matrix from the IMU sensor, including the current yaw and angular velocity.
     *
     * The measured state matrix is constructed using the robot's current yaw angle and angular velocity
     * obtained from the IMU. The values are stored in a 2x1 matrix for further processing in the Kalman filter.
     *
     * @return The measured state matrix containing the current orientation (yaw) and angular velocity (ω).
     */
    public Mat getMeasuredStateMatrix() {

        // Matrix to hold the measured state
        Mat measuredStateMatrix = new Mat(2, 1, CvType.CV_64FC1);

        // Retrieve the yaw angle and angular velocity from the IMU
        double[] measuredStateMatrixValues = new double[] {
                robotImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), // Current yaw angle in radians
                robotImu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate // Current angular velocity in radians
        };

        // Store the measured state values in the matrix
        measuredStateMatrix.put(0, 0, measuredStateMatrixValues);

        // Return the measured state matrix
        return measuredStateMatrix;
    }


    @Override
    public void periodic() {

    }
}
