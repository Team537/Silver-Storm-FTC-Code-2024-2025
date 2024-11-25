package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Rotation2d;

public class CoordinateSystem implements Subsystem {

    // Settings
    private final double PARALLEL_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS = 0; // TODO: Measure, from center of robot to center of ???
    private final double PERPENDICULAR_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS = 0; // TODO: Measure, from center of robot to center of ???

    // Hardware Storage
    private OdometryPod parallelOdometryPod;
    private OdometryPod perpendicularOdometryPod;

    // Storage Coordinate Data
    private Pose2d robotPosition;
    private double robotXCoordinate = 0;
    private double robotZCoordinate = 0;
    private double robotHeading = 0;

    // General Storage
    private Telemetry telemetry;

    /**
     * Create a new CoordinateSystem object.
     */
    public CoordinateSystem() {

        // Create this coordinate system's odometry pods.
        this.parallelOdometryPod = new OdometryPod("paraOdPod");
        this.parallelOdometryPod = new OdometryPod("perpOdPod");

        // Set the robot's position.
        this.robotPosition = new Pose2d();
        this.robotXCoordinate = 0;
        this.robotZCoordinate = 0;
        this.robotHeading = 0; // TODO: Do IMU stuff!
    }

    /**
     * Create a new CoordinateSystem object.
     */
    public CoordinateSystem(Pose2d startingPosition) {

        // Create this coordinate system's odometry pods.
        this.parallelOdometryPod = new OdometryPod("paraOdPod");
        this.parallelOdometryPod = new OdometryPod("perpOdPod");

        // Set the robot's position.
        this.robotPosition = startingPosition.clone();
        this.robotXCoordinate = startingPosition.getX();
        this.robotZCoordinate = startingPosition.getZ();
        this.robotHeading = startingPosition.getYawInRadians(); // TODO: Do IMU stuff!
    }

    /**
     * Initialize the CoordinateSystem subsystem. This sets up all of the odometry pods and allows for position to be measured.
     *
     * @param hardwareMap The OpMode's hardware map. This is required to access the robot's hardware.
     * @param telemetry The OpMode's telemetry.
     */
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup odometry pods.
        this.parallelOdometryPod.init(hardwareMap, telemetry);
        this.perpendicularOdometryPod.init(hardwareMap, telemetry);

        // Setup IMU stuff.
        // TODO: Add IMU stuff.

        // Store the robot's telemetry so that it can be used later.
        this.telemetry = telemetry;
    }

    /**
     * Return this CoordinateSystem's position as a Pose2d.
     *
     * @return This CoordinateSystem's position as a Pose2d.
     */
    public Pose2d getRobotPosition() {
        return this.robotPosition;
    }

    /**
     * Calculate the distance to the given position and return the result as a new Pose2d.
     *
     * @param position The position that the distance to will be found.
     * @return The distance to the given position and return the result as a new Pose2d.
     */
    public Pose2d getDistanceTo(Pose2d position) {
        return this.robotPosition.getDistanceTo(position);
    }

    /**
     * Returns this CoordinateSystem's x coordinate in meters.
     *
     * @return This CoordinateSystem's x coordinate in meters.
     */
    public double getRobotXCoordinateMeters() {
        return robotXCoordinate;
    }

    /**
     * Returns this CoordinateSystem's z coordinate in meters.
     *
     * @return This CoordinateSystem's z coordinate in meters.
     */
    public double getRobotZCoordinateMeters() {
        return robotZCoordinate;
    }

    /**
     * Returns this CoordinateSystem's heading in radians.
     *
     * @return This CoordinateSystem's heading in radians.
     */
    public double getRobotHeadingRadians() {
        return robotHeading;
    }

    /**
     * Updates the robot's position on the field based on the positional and rotational change in the
     * robot's odometry pods and IMU.
     */
    private void updatePosition() {

        // Get how far each odometry pod moved in meters.
        double parallelOdometryPodPositionalChange = parallelOdometryPod.getPositionalChangeMeters();
        double perpendicularOdometryPodPositionalChange = perpendicularOdometryPod.getPositionalChangeMeters();

        // Account for the change in position the encoders would experience from the robot rotating.
        double currentRobotHeadingRadians = 0; // TODO: IMPLEMENT IMU FEATURES!!!! Possibly move the change in theta to a separate class.
        double rotationalChangeRadians = currentRobotHeadingRadians - robotHeading;

        // Calculate the arc length each odometry wheel traveled. This is the distance the encoders traveled due to rotation.
        double parallelArcLengthMeters = PARALLEL_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS * rotationalChangeRadians;
        double perpendicularArcLengthMeters = PERPENDICULAR_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS * rotationalChangeRadians;

        // Calculate the local positional changes of each wheel.
        double localXChange = parallelOdometryPodPositionalChange - parallelArcLengthMeters;
        double localZChange = perpendicularOdometryPodPositionalChange - perpendicularArcLengthMeters;

        // Convert the positional changes to be relative to the field.
        double fieldXChange = (localXChange * Math.cos(currentRobotHeadingRadians)) - (localZChange * Math.sin(currentRobotHeadingRadians));
        double fieldZChange = (localXChange * Math.sin(currentRobotHeadingRadians)) + (localZChange * Math.cos(currentRobotHeadingRadians));

        // Update the robot's current position.
        this.robotXCoordinate += fieldXChange;
        this.robotZCoordinate += fieldZChange;
        this.robotHeading = currentRobotHeadingRadians;

        // Update the robot position object's values.
        this.robotPosition.setX(this.robotXCoordinate);
        this.robotPosition.setZ(this.robotZCoordinate);
        this.robotPosition.setYaw(this.robotHeading);
    }

    @Override
    public void periodic() {

        // Update the robot's position.
        updatePosition();

        // Call the odometry pod's periodic methods.
        parallelOdometryPod.periodic();
        perpendicularOdometryPod.periodic();

        // Display the robot's position on the telemetry.
        telemetry.addLine("Robot X (m): " + robotXCoordinate);
        telemetry.addLine("Robot Z (m): " + robotZCoordinate);
        telemetry.addLine("Robot Heading (°): " + (robotHeading / Math.PI * 180));
    }
}
