package org.firstinspires.ftc.teamcode.Subsystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utility.Controllers.PIDController;

import java.util.function.Supplier;

public class LinearSlides implements Subsystem {

    // Settings
    private final double ERROR_TOLLERANCE_METERS = 0.01;

    // Conversion Values
    private final double TICKS_TO_EXTENSION_DISTANCE_METERS = 0.00008792307 ;
    private Telemetry telemetry;

    // Hardware Storage
    private DcMotorEx slideMotor;

    // Supplier Storage
    private Supplier<Double> getArmAngleSupplier;

    // Controller Storage
    private PIDController pidController;
    private final double PROPORTIONAL_COEFFICIENT = 0;
    private final double INTEGRAL_COEFFICIENT = 0;
    private final double DERIVATIVE_COEFFICIENT = 0;
    private final double GRAVITATIONAL_COMPENSATION_CONSTANT = 0; // TODO: TUNE!!!!!!!

    // General Storage
    private SlideExtension targetLength;
    private double targetLengthEncoderTicks = 0;
    private double targetPosition = 0;

    // Flags
    private boolean eStopped = false;

    public LinearSlides(Supplier<Double> getArmAngleSupplier) {
        this.getArmAngleSupplier = getArmAngleSupplier;
    }

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
        this.pidController = new PIDController(PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT);
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
        double calculatedFeedbackMotorPower = this.pidController.update(this.targetPosition, this.slideMotor.getCurrentPosition());

        // Calculate feedforwards to compensate against gravity.
        double armAngle = this.getArmAngleSupplier.get();
        double feedforwards = GRAVITATIONAL_COMPENSATION_CONSTANT * Math.cos(armAngle);

        // Combine the PID values and feedforwards.
        double outputMotorPower = feedforwards + calculatedFeedbackMotorPower;

        // Set the motor power of the slide motor.
        this.slideMotor.setPower(outputMotorPower);
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
        this.targetLengthEncoderTicks = this.targetLength.getExtensionLengthMeters() / TICKS_TO_EXTENSION_DISTANCE_METERS;
    }

    @Deprecated
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

    @Override
    public void periodic() {

        // If the subsystem is emergency stopped, don't run any more code.
        if (this.eStopped) {
            return;
        }

        telemetry.addLine("Slide Position (m): " + getExtensionDistance());
        telemetry.addLine("Slide Position (in): " + getExtensionDistance() * 39.37);

        // Update motor velocity based on current system state.
        //calculateNewMotorPower();
    }
}
