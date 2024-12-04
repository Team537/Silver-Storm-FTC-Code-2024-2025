package org.firstinspires.ftc.teamcode.Subsystems.Odometry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

public class CoordinateSystem implements Subsystem {

    // Settings
    private final double PARALLEL_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS = -0.1138672902;
    private final double PERPENDICULAR_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS = -0.003570541649;

    // Hardware Storage
    private OdometryPod parallelOdometryPod;
    private OdometryPod perpendicularOdometryPod;
    private IMU imu;

    // Storage Coordinate Data
    private volatile Pose2d robotPosition;
    private volatile double robotXCoordinate = 0;
    private volatile double robotZCoordinate = 0;
    private volatile double robotHeading = 0;

    // Rotational Offsets
    private volatile double orientationOffset = 0;
    private volatile double driveRotationalOffset = 0;

    // General Storage
    private Telemetry telemetry;

    /**
     * Create a new CoordinateSystem object.
     */
    public CoordinateSystem() {

        // Create this coordinate system's odometry pods.
        this.parallelOdometryPod = new OdometryPod("paraOdPod");
        this.perpendicularOdometryPod = new OdometryPod("perpOdPod");

        // Set the robot's position.
        this.robotPosition = new Pose2d();
        this.robotXCoordinate = 0;
        this.robotZCoordinate = 0;

        this.robotHeading = 0;
        this.orientationOffset = 0;
    }

    /**
     * Create a new CoordinateSystem object.
     */
    public CoordinateSystem(Pose2d startingPosition) {

        // Create this coordinate system's odometry pods.
        this.parallelOdometryPod = new OdometryPod("paraOdPod");
        this.perpendicularOdometryPod = new OdometryPod("perpOdPod");

        // Set the robot's position.
        this.robotPosition = startingPosition.clone();
        this.robotXCoordinate = startingPosition.getX();
        this.robotZCoordinate = startingPosition.getZ();

        // Set an offset for the IMU so that it knows which direction it's facing.
        this.robotHeading = clampAngle(startingPosition.getYawInRadians());
        this.orientationOffset = clampAngle(startingPosition.getYawInRadians());
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

        // Create the IMU parameters that tell the IMU what direction its facing and allow us to use the
        // robot's rotation for various calculations.
        IMU.Parameters imuSettings = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuSettings);
        imu.resetYaw();

        // Store the robot's telemetry so that it can be used later.
        this.telemetry = telemetry;
    }

    /**
     * Sets an offset for the driver making it possible for them to drive in a field centric mode
     * relative to any angle they wish without interfering with the coordinate system or autonomous actions.
     */
    public void resetIMU() {
        this.driveRotationalOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Converts the given robot centric position to field space.
     *
     * @param objectPosition A position, relative to the robot's origin.
     * @return The given position, converted to field space.
     */
    public synchronized Pose2d robotSpaceToFieldSpace(Pose2d objectPosition) {

        double rotatedX = objectPosition.getX() * Math.cos(this.robotHeading) - objectPosition.getZ() * Math.sin(this.robotHeading);
        double rotatedZ = objectPosition.getX() * Math.sin(this.robotHeading) + objectPosition.getZ() * Math.cos(this.robotHeading);

        double objectFieldX = this.robotXCoordinate + rotatedX;
        double objectFieldZ = this.robotZCoordinate + rotatedZ;

        return new Pose2d(objectFieldX, objectFieldZ);
    }

    /**
     * Return this CoordinateSystem's position as a Pose2d.
     *
     * @return This CoordinateSystem's position as a Pose2d.
     */
    public synchronized Pose2d getRobotPosition() {
        return this.robotPosition;
    }

    /**
     * Calculate the distance to the given position and return the result as a new Pose2d.
     *
     * @param position The position that the distance to will be found.
     * @return The distance to the given position and return the result as a new Pose2d.
     */
    public synchronized Pose2d getDistanceTo(Pose2d position) {
        return this.robotPosition.getDistanceTo(position);
    }

    /**
     * Returns this CoordinateSystem's x coordinate in meters.
     *
     * @return This CoordinateSystem's x coordinate in meters.
     */
    public synchronized double getRobotXCoordinateMeters() {
        return this.robotXCoordinate;
    }

    /**
     * Returns this CoordinateSystem's z coordinate in meters.
     *
     * @return This CoordinateSystem's z coordinate in meters.
     */
    public synchronized double getRobotZCoordinateMeters() {
        return this.robotZCoordinate;
    }

    /**
     * Returns this CoordinateSystem's heading in radians.
     *
     * @return This CoordinateSystem's heading in radians.
     */
    public synchronized double getRobotHeadingRadians(boolean useDriverOffset) {

        // Get the angle from the IMU.
        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // If the rotation is being used for the driver, use the driver IMU offset. Otherwise, use
        // the orientation offset set by all autonomous functionality
        if (useDriverOffset) {
            robotHeading += driveRotationalOffset;
        } else {
            robotHeading += orientationOffset;
        }

        // Clamp the angle to within -pi and pi and return the robot's heading.
        return clampAngle(robotHeading);
    }

    /**
     * Clamp the given angle to within the given bounds.
     *
     * @param angleRadians The angle that needs to be clamped, in radians.
     * @return The clamped angle.
     */
    private double clampAngle(double angleRadians) {
        double outputAngleRadians = angleRadians;
        while (outputAngleRadians <= -Math.PI) {
            outputAngleRadians += 2 * Math.PI;
        }
        while (outputAngleRadians > Math.PI) {
            outputAngleRadians -= 2 * Math.PI;
        }
        return outputAngleRadians;
    }

    /**
     * Updates the robot's position on the field based on the positional and rotational change in the
     * robot's odometry pods and IMU.
     */
    private void updatePosition() {

        // Get how far each odometry pod moved in meters.
        double parallelOdometryPodPositionalChange = -parallelOdometryPod.getPositionalChangeMeters();
        double perpendicularOdometryPodPositionalChange = perpendicularOdometryPod.getPositionalChangeMeters();

        // Account for the change in position the encoders would experience from the robot rotating.
        double currentRobotHeadingRadians = getRobotHeadingRadians(false);
        double rotationalChangeRadians = currentRobotHeadingRadians - robotHeading;

        // Normalize the rotational change to the range [-PI, PI]
        if (rotationalChangeRadians <= -Math.PI) {
            rotationalChangeRadians += 2 * Math.PI;
        } else if (rotationalChangeRadians > Math.PI) {
            rotationalChangeRadians -= 2 * Math.PI;
        }

        // Calculate the arc length each odometry wheel traveled. This is the distance the encoders traveled due to rotation.
        double parallelArcLengthMeters = -PARALLEL_ODOMETRY_POD_DISTANCE_TO_CENTER_METERS * rotationalChangeRadians;
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

        // Call the odometry pod's periodic methods.
        parallelOdometryPod.periodic();
        perpendicularOdometryPod.periodic();

        // Update the robot's position.
        updatePosition();

        // Display the robot's position on the telemetry.
        telemetry.addLine("Robot X (m): " + robotXCoordinate);
        telemetry.addLine("Robot Z (m): " + robotZCoordinate);
        telemetry.addLine("Robot Heading (°): " + (robotHeading / Math.PI * 180));
    }
}