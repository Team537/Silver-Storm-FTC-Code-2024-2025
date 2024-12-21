package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;

public class LinearSlides implements Subsystem {

    // Settings
    private final double ERROR_TOLLERANCE_METERS = 0.01524;

    // Conversion Values
    private final double TICKS_TO_EXTENSION_DISTANCE_METERS = 0.00008604909 ;
    private Telemetry telemetry;

    // Hardware Storage
    private DcMotorEx slideMotor;

    // Controller Storage
    private PIDController pidController;
    private final double PROPORTIONAL_COEFFICIENT = 20;
    private final double INTEGRAL_COEFFICIENT = 0;
    private final double DERIVATIVE_COEFFICIENT = 0;

    // General Storage
    private SlideExtension targetLength;
    private double targetPosition = 0;

    // Flags
    private boolean eStopped = false;
    private boolean autonomousControlActive = false;

    /**
     * Sets up this subsystem so that it can function.
     *
     * @param hardwareMap The opMode's hardware map. This is required in order to gian access
     *                    to the robot's hardware.
     * @param telemetry The opMode's telemetry. This is required in order to output
     *                  diagnostic / feedback information.
     */
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Setup the PID Controller.
        this.pidController = new PIDController(PROPORTIONAL_COEFFICIENT, DERIVATIVE_COEFFICIENT, INTEGRAL_COEFFICIENT);
        this.pidController.setErrorTolerance(0.00127);
        this.targetLength = SlideExtension.REST;

        // Setup hardware.
        setupHardware(hardwareMap);

        // Store the telemetry for later use.
        this.telemetry = telemetry;
    }

    /**
     * Sets up the hardware for this subsystem.
     *
     * @param hardwareMap The hardware map so that hardware cna be accessed.
     */
    private void setupHardware(HardwareMap hardwareMap) {

        // Create te code object for the motor.
        this.slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        this.slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run the arm motor using the encoder so that we can use the encoders to preform various tasks.
        this.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops all hardware present within this subsystem.
     */
    public void eStop() {
        this.slideMotor.setPower(0);
        this.eStopped = true;
    }

    /**
     * Recalculate the motor's velocity based on the linear slides distance form the target position.
     */
    public void calculateNewMotorPower() {

        // Calculate feedback based on the distance towards the target position.
        double motorPower = this.pidController.update(this.targetPosition, getExtensionDistance());

        // Set the motor power of the slide motor.
        this.slideMotor.setPower(motorPower);
    }

    /**
     * Returns the distance the linear slides are extended.
     *
     * @return The distance the linear slides are extended.
     */
    public double getExtensionDistance() {
        return slideMotor.getCurrentPosition() * TICKS_TO_EXTENSION_DISTANCE_METERS;
    }

    /**
     * Sets the target length for the slide to reach.
     *
     * @param targetLength The length the slide will target.
     */
    public void setTargetLength(SlideExtension targetLength) {
        this.targetLength = targetLength;
        this.targetPosition = targetLength.getExtensionLengthMeters();
    }

    /**
     * Sets the target length of the linear slides, in inches.
     *
     * @param targetLengthInches The target length of the linear slides, in inches.
     */
    public void setTargetLengthInches(double targetLengthInches) {
        this.targetPosition = targetLengthInches / 39.37;
    }

    /**
     * Sets the target length of the linear slides, in meters.
     *
     * @param targetLengthMeters The target length of the linear slides, in meters.
     */
    public void setTargetLengthMeters(double targetLengthMeters) {
        this.targetPosition = targetLengthMeters;
    }

    /**
     * Sets the motor power of the slide motor allowing for human controlled slide movement.
     *
     * @param motorPower The power the slide motor will run at.
     */
    public void setMotorPower(double motorPower) {
        this.slideMotor.setPower(motorPower);
    }

    /**
     * Return whether or not the linear slides have reached their target position.
     *
     * @return Whether or not the linear slides have reached their target position.
     */
    public boolean atTargetPosition() {
        return Math.abs(this.slideMotor.getCurrentPosition()) - Math.abs(this.targetPosition) <= ERROR_TOLLERANCE_METERS;
    }

    /**
     * Toggles the autonomous control of this subsystem.
     *
     * @param active Whether or not the robot can be controlled autonomously.
     */
    public void toggleAutonomousControl(boolean active) {
        this.autonomousControlActive = active;
    }

    @Override
    public void periodic() {

        // If the subsystem is emergency stopped, don't run any more code.
        if (this.eStopped) {
            return;
        }

        //telemetry.addLine("Slide Position: " + slideMotor.getCurrentPosition());
        //telemetry.addLine("Slide Position (m): " + getExtensionDistance());
        telemetry.addLine("Slide Position (in): " + getExtensionDistance() * 39.37);
        telemetry.addLine("Slides At Target: " + atTargetPosition());

        // Update motor velocity based on current system state.
        if (autonomousControlActive) {
            calculateNewMotorPower();
        }
    }
}
