package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.CoordinateSystem;
import org.firstinspires.ftc.teamcode.Utility.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Utility.Controllers.RotationalPIDController;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

public class Drivetrain implements Subsystem{

    // Subsystems
    private CoordinateSystem coordinateSystem;

    // Hardware
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;

    // PID Controllers
    private RotationalPIDController rotationalPIDController;
    private PIDController xPIDController;
    private PIDController zPIDController;

    // PID Coefficients
    private final double ROTATIONAL_PROPORTIONAL_COEFFICIENT = 0.5;
    private final double ROTATIONAL_DERIVATIVE_COEFFICIENT = 0.005;
    private final double ROTATIONAL_INTEGRAL_COEFFICIENT = 0;

    private final double POSITIONAL_PROPORTIONAL_COEFFICIENT = 1.25;
    private final double POSITIONAL_DERIVATIVE_COEFFICIENT = 0.005;
    private final double POSITIONAL_INTEGRAL_COEFFICIENT = 0;

    // Target Storage
    private Pose2d targetPosition = new Pose2d();

    // Settings
    private boolean fieldCentricEnabled = false;
    private boolean velocityDriveEnabled = false;
    private double rotationalErrorToleranceRadians = 0.0174533;
    private double positionalErrorToleranceMeters = 0.01016;

    // Flags
    private boolean autonomousControlActive = false;

    // Storage
    private Pose2d robotPosition;
    private Telemetry telemetry;

    /**
     * Creates a new drivetrain, with the default starting position.
     */
    public Drivetrain() {
        this.robotPosition = new Pose2d();
        this.coordinateSystem = new CoordinateSystem();
    }

    /**
     * Create a new drivetrain, with its position set to the provided Pose2d.
     *
     * @param startingLocation The robot's position on the field, as a Pose2d.
     */
    public Drivetrain(Pose2d startingLocation) {
        this.robotPosition = startingLocation;
        this.coordinateSystem = new CoordinateSystem(startingLocation);
    }

    /**
     * Sets up the drivetrain so that it can function.
     *
     * @param hardwareMap The opMode's hardware map. This is required in order to gian access
     *                          to the robot's hardware.
     * @param telemetry The opMode's telemetry. This is required in order to output
     *                        diagnostic / feedback information.
     */
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup the robot's hardware.
        setupHardware(hardwareMap);

        // Store the telemetry for future use.
        this.telemetry = telemetry;

        // Create PID Controller Objects.
        this.rotationalPIDController = new RotationalPIDController(ROTATIONAL_PROPORTIONAL_COEFFICIENT, ROTATIONAL_INTEGRAL_COEFFICIENT, ROTATIONAL_DERIVATIVE_COEFFICIENT);
        this.rotationalPIDController.setErrorTolerance(rotationalErrorToleranceRadians);

        this.xPIDController = new PIDController(POSITIONAL_PROPORTIONAL_COEFFICIENT, POSITIONAL_INTEGRAL_COEFFICIENT, POSITIONAL_DERIVATIVE_COEFFICIENT);
        this.xPIDController.setErrorTolerance(positionalErrorToleranceMeters);

        this.zPIDController = new PIDController(POSITIONAL_PROPORTIONAL_COEFFICIENT, POSITIONAL_INTEGRAL_COEFFICIENT, POSITIONAL_DERIVATIVE_COEFFICIENT);
        this.zPIDController.setErrorTolerance(positionalErrorToleranceMeters);

        // Enable the coordinate system
        coordinateSystem.init(hardwareMap, telemetry);
    }

    /**
     * Sets up the robot's hardware.
     *
     * @param hardwareMap The opMode's hardware map. This is required in order to gian access
     *                          to the robot's hardware.
     */
    private void setupHardware(@NonNull HardwareMap hardwareMap) {

        // Setup the drive motors.
        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "frontRightDriveMotor");
        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "frontLeftDriveMotor");
        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "backRightDriveMotor");
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "backLeftDriveMotor");

        // Setup the drive motors.
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run all motors using RUN_WITH_ENCODER so that we can use encoder related methods.
        // Commented out for bug testing purposes.
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Resets the IMU such that the robot believes it is facing the zero direction.
     */
    public void resetIMU() {
        this.coordinateSystem.resetIMU();
    }

    /**
     * Preforms necessary calculations required to drive the robot in the desired direction at the
     * desired speed.
     *
     * @param xInput The controller input correlated to motion along the X axis.
     * @param zInput The controller input correlated to motion along the Z axis.
     * @param rotationalInput The controller input correlated to rotational motion.
     */
    public void driveRobotWithControllerInputs(double xInput, double zInput, double rotationalInput) {

        // Keep track of the robot's velocity in each direction.
        double xMotion = xInput;
        double zMotion = zInput;
        double rotationalMotion = rotationalInput;

        // If the robot is instructed to drive in a field centric manner, rotate the inputs.
        if (fieldCentricEnabled ) {

            // Store the robot's rotation.
            double robotYawRadians = coordinateSystem.getRobotHeadingRadians(true);

            // Rotate the velocity value
            xMotion = (xInput * Math.cos(-robotYawRadians)) - (zInput * Math.sin(-robotYawRadians));
            zMotion = (xInput * Math.sin(-robotYawRadians)) + (zInput * Math.cos(-robotYawRadians));
        }

        // Counteract Imperfect Strafing.
        xMotion *= DrivetrainConstants.STRAIF_OFFSET_MULTIPLIER;

        // Drive the robot in the desired direction using the adjusted values.
        driveRobot(xMotion, zMotion, rotationalInput);
    }

    /**
     * Converts speeds into motor inputs so that the robot can drive.
     *
     * @param xMotion The speed the robot will drive along the X axis.
     * @param zMotion The speed the robot will drive along the Z axis.
     * @param rotationalMotion The speed the robot will rotate.
     */
    public void driveRobot(double xMotion, double zMotion, double rotationalMotion) {

        // The scalingFactor is the largest motor power / velocity.
        // The motor power / velocity cannot exceed a specific value, so it needs to be scaled down
        // in order to maintain a consistent ratio between each motor.
        double scalingFactor;
        double absoluteMotionSum = Math.abs(xMotion) + Math.abs(zMotion) + Math.abs(rotationalMotion);

        // Calculate the denominator differently based on whether or not the robot is driving using velocity.
        scalingFactor = Math.max(absoluteMotionSum, DrivetrainConstants.MAX_MOTOR_POWER);

        // If we are using velocity drive, convert the motor power value to velocity.
        if(velocityDriveEnabled) {
            xMotion *= DrivetrainConstants.MAX_MOTOR_TICKS_PER_SECOND;
            zMotion *= DrivetrainConstants.MAX_MOTOR_TICKS_PER_SECOND;
            rotationalMotion *= DrivetrainConstants.MAX_MOTOR_TICKS_PER_SECOND;
        }

        // Calculate the motion for each individual motor.
        double frontRightSpeed = (zMotion - xMotion - rotationalMotion) / scalingFactor;
        double backRightSpeed = (zMotion + xMotion - rotationalMotion) / scalingFactor;
        double frontLeftSpeed = (zMotion + xMotion + rotationalMotion) / scalingFactor;
        double backLeftSpeed = (zMotion - xMotion + rotationalMotion) / scalingFactor;

        // If the robot is driving using velocity, set the velocity of each motor.
        // Otherwise, set the motor power of each motor.
        if (velocityDriveEnabled) {
            setMotorVelocity(frontRightSpeed, backRightSpeed, frontLeftSpeed, backLeftSpeed);
        } else {
            setMotorPower(frontRightSpeed, backRightSpeed, frontLeftSpeed, backLeftSpeed);
        }
    }

    /**
     * Sets each motor to the specified velocity.
     *
     * @param frontRightVelocity The velocity that the front right motor will be set to.
     * @param backRightVelocity The velocity that the back right motor will be set to.
     * @param frontLeftVelocity The velocity that the front left motor will be set to.
     * @param backLeftVelocity The velocity that the back left motor will be set to.
     */
    private void setMotorVelocity(double frontRightVelocity, double backRightVelocity, double frontLeftVelocity, double backLeftVelocity) {

        // Set the velocity for each motor.
        frontRightDriveMotor.setVelocity(frontRightVelocity);
        backRightDriveMotor.setVelocity(backRightVelocity);
        frontLeftDriveMotor.setVelocity(frontLeftVelocity);
        backLeftDriveMotor.setVelocity(backLeftVelocity);
    }

    /**
     * Sets each motor to the specified power.
     *
     * @param frontRightPower The power that the front right motor will be set to.
     * @param backRightPower The power that the back right motor will be set to.
     * @param frontLeftPower The power that the front left motor will be set to.
     * @param backLeftPower The power that the back left motor will be set to.
     */
    private void setMotorPower(double frontRightPower, double backRightPower, double frontLeftPower, double backLeftPower) {

        // Set the velocity for each motor.
        frontRightDriveMotor.setPower(frontRightPower);
        backRightDriveMotor.setPower(backRightPower);
        frontLeftDriveMotor.setPower(frontLeftPower);
        backLeftDriveMotor.setPower(backLeftPower);
    }

    /**
     * Toggles the robot's field centric state. E.g. if the robot is currently driving in a field centric
     * manner, it will now drive in a robot centric manner.
     */
    public void toggleFieldCentricDrive() {
        fieldCentricEnabled = !fieldCentricEnabled;
    }

    /**
     * Toggles whether or not the robot is driving with velocity or motor power.
     */
    public void toggleVelocityDrive() {
        velocityDriveEnabled = !velocityDriveEnabled;
    }

    /**
     * Sets the robot's field centric state to the specified state.
     *
     * @param enabled Whether or not the robot will drive in a field centric manner.
     */
    public void toggleFieldCentricDrive(boolean enabled) {
        this.fieldCentricEnabled = enabled;
    }

    /**
     * Sets whether or not the robot will drive using velocity or motor power.
     *
     * @param enabled Whether or not the robot will drive using velocity.
     */
    public void toggleVelocityDrive(boolean enabled) {
        velocityDriveEnabled = enabled;
    }

    /**
     * Returns this robot's coordinate system.
     *
     * @return This robot's coordinate system.
     */
    public Subsystem getCoordinateSystem() {
        return this.coordinateSystem;
    }

    /**
     * Returns this robot's position as a pose2d.
     *
     * @return This robot's position as a pose2d.
     */
    public Pose2d getRobotPosition() {
        return this.coordinateSystem.getRobotPosition();
    }

    /**
     * Returns whether or not the robot is driving in a field centric manner.
     *
     * @return Whether or not the robot is driving in a field centric manner.
     */
    public boolean getFieldCentricEnabled() {
        return fieldCentricEnabled;
    }

    /**
     * Returns whether or not the robot is driving using velocity.
     *
     * @return Whether or not the robot is driving using velocity.
     */
    public boolean getVelocityDriveEnabled() {
        return velocityDriveEnabled;
    }

    /**
     * Toggles whether or not this device can operate autonomously.
     *
     * @param autonomousControlActive Whether or not the robot can drive autonomously.
     */
    public void toggleAutonomousControl(boolean autonomousControlActive) {
        this.autonomousControlActive = autonomousControlActive;
    }

    /**
     * Sets the drivetrain's target position, as a Pose2d.
     *
     * @param targetPosition The drivetrain's target position, as a Pose2d.
     */
    public void setTargetPosition(Pose2d targetPosition) {
        this.targetPosition = targetPosition;
        rotationalPIDController.reset();
        xPIDController.reset();
        zPIDController.reset();
    }

    /**
     * Sets the drivetrain's target rotation, in degrees.
     *
     * @param thetaDegrees The drivetrain's target rotation, in degrees.
     */
    public void setTargetRotation(double thetaDegrees) {
        this.targetPosition.setYaw(Math.toRadians(thetaDegrees));
        rotationalPIDController.reset();
    }

    private void calculateNewMotorPower() {

        // Store the robot's rotation.
        double robotYawRadians = coordinateSystem.getRobotHeadingRadians(false);

        // Calculate the necessary motor powers to drive the robot.
        double rotationalMotion = -this.rotationalPIDController.update(this.targetPosition.getYawInRadians(), robotYawRadians);
        double xMotion = this.xPIDController.update(this.targetPosition.getX(), coordinateSystem.getRobotXCoordinateMeters());
        double zMotion = this.zPIDController.update(this.targetPosition.getZ(), coordinateSystem.getRobotZCoordinateMeters());

        // Rotate the velocity value
        double rotatedXMotion = (xMotion * Math.cos(-robotYawRadians)) - (zMotion * Math.sin(-robotYawRadians));
        double rotatedZMotion = (xMotion * Math.sin(-robotYawRadians)) + (zMotion * Math.cos(-robotYawRadians));

        // Account for imperfect straifing.
        rotatedXMotion *= 1.1;

        // The scalingFactor is the largest motor power / velocity.
        // The motor power / velocity cannot exceed a specific value, so it needs to be scaled down
        // in order to maintain a consistent ratio between each motor.
        double scalingFactor;
        double absoluteMotionSum = Math.abs(rotatedXMotion) + Math.abs(rotatedZMotion) + Math.abs(rotationalMotion);

        // Calculate the denominator differently based on whether or not the robot is driving using velocity.
        scalingFactor = Math.max(absoluteMotionSum, DrivetrainConstants.MAX_MOTOR_POWER);

        // Calculate the motion for each individual motor.
        double frontRightSpeed = (rotatedXMotion - rotatedZMotion - rotationalMotion) / scalingFactor;
        double backRightSpeed = (rotatedXMotion + rotatedZMotion - rotationalMotion) / scalingFactor;
        double frontLeftSpeed = (rotatedXMotion + rotatedZMotion + rotationalMotion) / scalingFactor;
        double backLeftSpeed = (rotatedXMotion - rotatedZMotion + rotationalMotion) / scalingFactor;

        // Set the robot's motor powers.
        setMotorPower(frontRightSpeed, backRightSpeed, frontLeftSpeed, backLeftSpeed);
    }

    /**
     * Returns whether or not the robot is at the target position.
     *
     * @return Whether or not the robot is at the target position.
     */
    public boolean atTargetPosition() {

        // Check if the robot is within the error tolerance for its target position. If it isn't return false,
        // as we aren't yet at the target position.
        if (Math.abs(coordinateSystem.getRobotXCoordinateMeters() - this.targetPosition.getX()) > positionalErrorToleranceMeters) {
            return false;
        }
        if (Math.abs(coordinateSystem.getRobotZCoordinateMeters() - this.targetPosition.getZ()) > positionalErrorToleranceMeters) {
            return false;
        }

        // Return whether or not the robot is within the rotational tolerance. This i the only thing left
        // determining if the robot is at the target position.
        return !(Math.abs(coordinateSystem.clampAngle(coordinateSystem.getRobotHeadingRadians(false) - this.targetPosition.getYawInRadians())) < rotationalErrorToleranceRadians);
    }

    @Override
    public void periodic() {

        // Call other subsystem's periodic methods.
        coordinateSystem.periodic();

        // Output diagnostic data.
        telemetry.addLine("Target X (m): " + targetPosition.getX());
        telemetry.addLine("Target Z (m): " + targetPosition.getZ());
        telemetry.addLine("Target Rotation: " + (targetPosition.getYawInRadians() * 180 / Math.PI));
        telemetry.addLine("Active : " + autonomousControlActive);

        // If told to do so, drive autonomously.
        if (autonomousControlActive) {
            calculateNewMotorPower();
        }
    }
}