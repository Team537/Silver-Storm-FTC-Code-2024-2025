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
import org.firstinspires.ftc.teamcode.Utility.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

public class Drivetrain implements Subsystem{

    // Hardware
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private IMU imu;

    // Settings
    private boolean fieldCentricEnabled = false;
    private boolean velocityDriveEnabled = false;

    // Flags
    private boolean isSetup = false;

    // Storage
    private Pose2d robotPosition;
    private Telemetry telemetry;

    /**
     * Creates a new drivetrain, with the default starting position.
     */
    public Drivetrain() {
        robotPosition = new Pose2d();
    }

    /**
     * Create a new drivetrain, with its position set to the provided Pose2d.
     *
     * @param startingLocation The robot's position on the field, as a Pose2d.
     */
    public Drivetrain(Pose2d startingLocation) {
        robotPosition = startingLocation;
    }

    /**
     * Sets up the robot's hardware.
     *
     * @param opModeHardwareMap The opMode's hardware map. This is required in order to gian access
     *                          to the robot's hardware.
     */
    private void setupHardware(@NonNull HardwareMap opModeHardwareMap) {

        // Setup the drive motors.
        frontRightDriveMotor = opModeHardwareMap.get(DcMotorEx.class, "frontRightDriveMotor");
        frontLeftDriveMotor = opModeHardwareMap.get(DcMotorEx.class, "frontLeftDriveMotor");
        backRightDriveMotor = opModeHardwareMap.get(DcMotorEx.class, "backRightDriveMotor");
        backLeftDriveMotor = opModeHardwareMap.get(DcMotorEx.class, "backLeftDriveMotor");

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

        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Create the IMU parameters that tell the IMU what direction its facing and allow us to use the
        // robot's rotation for various calculations.
        IMU.Parameters imuSettings = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // Initialize the IMU
        imu = opModeHardwareMap.get(IMU.class, "imu");
        imu.initialize(imuSettings);
    }

    /**
     * Sets up the drivetrain so that it can function.
     * @param opModeHardwareMap The opMode's hardware map. This is required in order to gian access
     *                          to the robot's hardware.
     * @param opModeTelemetry The opMode's telemetry. This is required in order to output
     *                        diagnostic / feedback information.
     */
    @Override
    public void init(HardwareMap opModeHardwareMap, Telemetry opModeTelemetry) {

        // Setup the robot's hardware.
        setupHardware(opModeHardwareMap);

        // Set the robot's position.
        robotPosition = new Pose2d();
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
            double robotYawRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the velocity value
            xMotion = (xInput * Math.cos(-robotYawRadians)) - (zInput * Math.sin(-robotYawRadians));
            zMotion = (xInput * Math.sin(-robotYawRadians)) + (zInput * Math.cos(-robotYawRadians));
        }

        // If velocityDrive is enabled, convert each motion value into a velocity value.
        if (velocityDriveEnabled) {

            // Get the max speed that each motor can turn and save it as a variable. his is done to
            // improve code readability.
            double maxAngularVelocity = DrivetrainConstants.MAX_MOTOR_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

            // Convert each value into a velocity value.
            xMotion *= maxAngularVelocity;
            zMotion *= maxAngularVelocity;
            rotationalMotion *= maxAngularVelocity;
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
        if (velocityDriveEnabled) {
            scalingFactor = Math.max(absoluteMotionSum, DrivetrainConstants.MAX_MOTOR_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        } else {
            scalingFactor = Math.max(absoluteMotionSum, DrivetrainConstants.MAX_MOTOR_POWER);
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
     * Returns an array containing the current velocity state of each of this robot's motors.
     *
     * @return An array containing the velocity of each motor. Ordered as follows: FR, FL, BL, BR
     */
    public double[] getVelocityState() {

        // Create a new array to store the velocity of each motor.
        double[] velocityState = new double[4];
        velocityState[0] = this.frontRightDriveMotor.getVelocity();
        velocityState[1] = this.frontLeftDriveMotor.getVelocity();
        velocityState[2] = this.backLeftDriveMotor.getVelocity();
        velocityState[3] = this.backRightDriveMotor.getVelocity();

        // Return the velocities of the motors.
        return velocityState;
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

    @Override
    public void periodic() {

    }
}