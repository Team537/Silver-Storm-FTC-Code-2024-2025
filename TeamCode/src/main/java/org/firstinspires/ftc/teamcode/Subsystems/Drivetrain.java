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
import org.firstinspires.ftc.teamcode.Utility.Constants;
import org.firstinspires.ftc.teamcode.Utility.Geometry.Pose2d;

public class Drivetrain implements Subsystem{

    // Hardware
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private IMU imu;

    // Flags
    private boolean isSetup = false;

    // Storage
    private Pose2d robotPosition;
    private Telemetry telemetry;

    /**
     * Sets up the robot's hardware.
     *
     * @param opModeHardwareMap The opMode's hardware map. This is required in order to gian access
     *                          to the robot's hardware.
     */
    private void setupHardware(@NonNull HardwareMap opModeHardwareMap) {

        // Setup the drive motors.
        frontRightDriveMotor = opModeHardwareMap.get(DcMotorEx.class, "frontRightDriveMotor");
        frontLeftDriveMotor = opModeHardwareMap.get(DcMotorEx.class, "frontRightDriveMotor");
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
     * Sets up the drivetrain so that it can function.
     * @param opModeHardwareMap The opMode's hardware map. This is required in order to gian access
     *                          to the robot's hardware.
     * @param opModeTelemetry The opMode's telemetry. This is required in order to output
     *                        diagnostic / feedback information.
     * @param startingPosition  The starting position of the robot, as a Pose2d.
     */
    public void init(HardwareMap opModeHardwareMap, Telemetry opModeTelemetry, Pose2d startingPosition) {

        // Setup the robot's hardware.
        setupHardware(opModeHardwareMap);

        // Set the robot's position.
        robotPosition = startingPosition;
    }

    /**
     * Converts speeds into motor inputs so that the robot can drive.
     *
     * @param xSpeed The speed the robot will drive along the X axis.
     * @param zSpeed The speed the robot will drive along the Z axis.
     * @param rotationalSpeed The speed the robot will rotate.
     */
    public void driveRobotWithMotorPower(double xSpeed, double zSpeed, double rotationalSpeed) {

        // The denominator is the largest motor power.
        // Motor power cannot exceed 1, so we need to divide the values to ensure that the motors
        // maintain the same ration.
        double denominator = Math.max(Math.abs(xSpeed) + Math.abs(xSpeed) + Math.abs(zSpeed) + Math.abs(rotationalSpeed), 1);

        // Calculate the power for each individual motor.
        double frontRightSpeed = (zSpeed - xSpeed - rotationalSpeed) / denominator;
        double backRightSpeed = (zSpeed + xSpeed - rotationalSpeed) / denominator;
        double frontLeftSpeed = (zSpeed + xSpeed + rotationalSpeed) / denominator;
        double backLeftSpeed = (zSpeed - xSpeed + rotationalSpeed) / denominator;

        // Set the power for each motor.
        frontRightDriveMotor.setPower(frontRightSpeed);
        backRightDriveMotor.setPower(backRightSpeed);
        frontLeftDriveMotor.setPower(frontLeftSpeed);
        backLeftDriveMotor.setPower(backLeftSpeed);
    }

    /**
     * Preforms necessary calculations required to drive the robot in the desired direction at the
     * desired speed.
     *
     * @param xInput The controller input correlated to motion along the X axis.
     * @param zInput The controller input correlated to motion along the Z axis.
     * @param rotationalInput The controller input correlated to rotational motion.
     * @param fieldCentric Whether or no the robot should drive in a field centric manner.
     */
    public void driveRobotWithControllerInputs(double xInput, double zInput, double rotationalInput, boolean fieldCentric) {

        // Keep track of the robot's velocity in each direction.
        double xVelocityMeters = xInput;
        double zVelocityMeters = zInput;

        // If the robot is instructed to drive in a field centric manner, rotate the inputs.
        if (fieldCentric) {

            // Store the robot's rotation.
            double robotYawRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the velocity value
            xVelocityMeters = (xVelocityMeters * Math.cos(-robotYawRadians)) - (zVelocityMeters * Math.sin(-robotYawRadians));
            zVelocityMeters = (xVelocityMeters * Math.sin(-robotYawRadians)) + (zVelocityMeters * Math.cos(-robotYawRadians));
        }

        // Counteract Imperfect Strafing.
        xVelocityMeters *= Constants.DrivetrainConstants.STRAIF_OFFSET_MULTIPLIER;

        // Drive the robot in the desired direction using the adjusted values.
        driveRobotWithMotorPower(xVelocityMeters, zVelocityMeters, rotationalInput);
    }

    @Override
    public void periodic() {

    }
}
